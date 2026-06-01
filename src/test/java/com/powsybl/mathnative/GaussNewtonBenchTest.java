package com.powsybl.mathnative;

import com.powsybl.math.solver.GaussNewtonCholesky;
import com.powsybl.math.solver.GaussNewtonKLU;
import com.powsybl.math.solver.NativeBuffers;
import org.junit.jupiter.api.Test;

import java.nio.DoubleBuffer;
import java.nio.IntBuffer;
import java.util.Arrays;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Larger-scale sparse Gauss-Newton test, used as a measurement harness to spot
 * remaining bottlenecks. Generates a sparse linear least-squares problem with
 * a known solution, then runs many Gauss-Newton iterations through each solver
 * and prints per-iteration timings.
 *
 * The problem: y = H x* + noise, with H having M rows and N columns. Each row
 * has exactly K random nonzero entries, so Ht (which is what the solver takes)
 * has K entries per column. The Jacobian pattern is fixed across iterations,
 * which is the regime the pattern cache is designed for.
 */
class GaussNewtonBenchTest {

    // Banded sizing targets a 10000-bus AC state estimation: 2 unknowns/bus,
    // typical 2-3x measurement redundancy, K ~= 8 for power-injection rows.
    // Random sizing is kept small because fully-random sparsity makes factor
    // cost explode (random patterns are pathological for sparse Cholesky/LU).
    private static final int N_BAND = 20_000;
    private static final int M_BAND = 60_000;
    private static final int K_BAND = 8;
    private static final int N_RAND = 1_000;
    private static final int M_RAND = 10_000;
    private static final int K_RAND = 6;
    private static final int ITERS = 15;

    private enum Pattern {
        /** Each obs picks K independent random columns - pathological for factorization. */
        RANDOM,
        /** Each obs picks K consecutive columns in a window glided across [0, N). Banded H/C. */
        BANDED
    }

    private static class Problem {
        int n;
        int m;
        int[] ap;
        int[] ai;
        double[] ax;
        double[] y;
        double[] sqrtWeights;
        double[] trueX;
    }

    private static Problem buildProblem(int n, int m, int k, Pattern pattern, long seed) {
        Random rng = new Random(seed);
        Problem p = new Problem();
        p.n = n;
        p.m = m;
        p.trueX = new double[n];
        for (int i = 0; i < n; i++) {
            p.trueX[i] = rng.nextGaussian();
        }

        int nnz = m * k;
        p.ap = new int[m + 1];
        p.ai = new int[nnz];
        p.ax = new double[nnz];
        p.y = new double[m];
        p.sqrtWeights = new double[m];
        Arrays.fill(p.sqrtWeights, 1.0);

        int[] cols = new int[k];
        boolean[] picked = new boolean[n];
        int idx = 0;
        for (int row = 0; row < m; row++) {
            p.ap[row] = idx;
            switch (pattern) {
                case RANDOM: {
                    int filled = 0;
                    while (filled < k) {
                        int c = rng.nextInt(n);
                        if (!picked[c]) {
                            picked[c] = true;
                            cols[filled++] = c;
                        }
                    }
                    for (int c : cols) {
                        picked[c] = false;
                    }
                    Arrays.sort(cols);
                    break;
                }
                case BANDED: {
                    // Slide a window of width k across [0, n) as row goes from 0 to m-1.
                    int start = (int) ((long) row * (n - k) / Math.max(1, m - 1));
                    for (int j = 0; j < k; j++) {
                        cols[j] = start + j;
                    }
                    break;
                }
            }

            double sum = 0.0;
            for (int j = 0; j < k; j++) {
                double v = rng.nextGaussian();
                p.ai[idx] = cols[j];
                p.ax[idx] = v;
                sum += v * p.trueX[cols[j]];
                idx++;
            }
            p.y[row] = sum + 0.01 * rng.nextGaussian();
        }
        p.ap[m] = idx;
        return p;
    }

    @FunctionalInterface
    private interface InitFn {
        void apply(String id, int m, int n, double[] wDiag);
    }

    @FunctionalInterface
    private interface SolveFn {
        void apply(String id, DoubleBuffer r, int m, int n,
                   IntBuffer ap, IntBuffer ai, DoubleBuffer ax, DoubleBuffer result);
    }

    @FunctionalInterface
    private interface ReleaseFn {
        void apply(String id);
    }

    private static void runBench(String label, Pattern pattern, int n, int m, int k,
                                 InitFn init, SolveFn solve, ReleaseFn release) {
        Problem p = buildProblem(n, m, k, pattern, 42L);
        long[] perIter = new long[ITERS];
        String id = "bench-" + label;
        init.apply(id, p.m, p.n, p.sqrtWeights);
        try {
            double[] params = new double[p.n];
            double[] delta = new double[p.n];
            double[] residuals = new double[p.m];

            // Allocate direct buffers once; reuse across iterations. ap/ai/ax never
            // change (linear model) so populate them up-front.
            IntBuffer apBuf = NativeBuffers.wrap(p.ap);
            IntBuffer aiBuf = NativeBuffers.wrap(p.ai);
            DoubleBuffer axBuf = NativeBuffers.wrap(p.ax);
            DoubleBuffer rBuf = NativeBuffers.allocDouble(p.m);
            DoubleBuffer deltaBuf = NativeBuffers.allocDouble(p.n);

            for (int iter = 0; iter < ITERS; iter++) {
                // r = y - H * params; linear model so p.ax is reused unchanged.
                System.arraycopy(p.y, 0, residuals, 0, p.m);
                for (int row = 0; row < p.m; row++) {
                    double pred = 0.0;
                    for (int q = p.ap[row]; q < p.ap[row + 1]; q++) {
                        pred += p.ax[q] * params[p.ai[q]];
                    }
                    residuals[row] -= pred;
                }
                NativeBuffers.copyFrom(rBuf, residuals);

                long t0 = System.nanoTime();
                solve.apply(id, rBuf, p.m, p.n, apBuf, aiBuf, axBuf, deltaBuf);
                perIter[iter] = System.nanoTime() - t0;

                NativeBuffers.copyTo(deltaBuf, delta);
                for (int i = 0; i < p.n; i++) {
                    params[i] += delta[i];
                }
            }
            reportTimings(label + " [" + pattern + "]", p.n, p.m, k, perIter);

            // Convergence is to the noisy-data optimum; expected L2 error scales like
            // noise * N / sqrt(M*K) for an oversampled LS. Use a generous bound to
            // confirm we hit a sensible answer.
            double bound = 0.05 * Math.sqrt(p.n);
            double err = l2(params, p.trueX);
            System.out.printf("  %s ||params - trueX||_2 = %.6f (bound %.3f)%n", label, err, bound);
            assertTrue(err < bound, label + " did not converge: err=" + err + " bound=" + bound);
        } finally {
            release.apply(id);
        }
    }

    @Test
    void benchCholeskyRandom() {
        GaussNewtonCholesky s = new GaussNewtonCholesky();
        runBench("Cholesky-rand", Pattern.RANDOM, N_RAND, M_RAND, K_RAND,
                s::init, s::solve, s::release);
    }

    @Test
    void benchCholeskyBanded() {
        GaussNewtonCholesky s = new GaussNewtonCholesky();
        runBench("Cholesky-band", Pattern.BANDED, N_BAND, M_BAND, K_BAND,
                s::init, s::solve, s::release);
    }

    @Test
    void benchKluRandom() {
        GaussNewtonKLU s = new GaussNewtonKLU();
        runBench("KLU-rand", Pattern.RANDOM, N_RAND, M_RAND, K_RAND,
                s::init, s::solve, s::release);
    }

    @Test
    void benchKluBanded() {
        GaussNewtonKLU s = new GaussNewtonKLU();
        runBench("KLU-band", Pattern.BANDED, N_BAND, M_BAND, K_BAND,
                s::init, s::solve, s::release);
    }

    /**
     * Mimics state-estimation bad-data analysis: factor the gain matrix once,
     * then solve repeatedly with perturbed residuals (e.g. measurement i
     * removed, or residual scaled). Each solveFactorized() should cost only
     * rhs + linear-solve - factorization is the cached big win.
     */
    @FunctionalInterface
    private interface FactorizeFn {
        int apply(String id, int m, int n, IntBuffer ap, IntBuffer ai, DoubleBuffer ax);
    }

    @FunctionalInterface
    private interface SolveFactorizedFn {
        void apply(String id, DoubleBuffer r, DoubleBuffer result);
    }

    private static void runMultiRhsBench(String label,
                                         InitFn init, FactorizeFn factorize,
                                         SolveFactorizedFn solveCached, ReleaseFn release) {
        // Use the banded pattern at transmission scale (random would dominate cold
        // factor time so much that the warm story is lost).
        Problem p = buildProblem(N_BAND, M_BAND, K_BAND, Pattern.BANDED, 42L);
        String id = "bench-" + label;
        init.apply(id, p.m, p.n, p.sqrtWeights);
        try {
            IntBuffer apBuf = NativeBuffers.wrap(p.ap);
            IntBuffer aiBuf = NativeBuffers.wrap(p.ai);
            DoubleBuffer axBuf = NativeBuffers.wrap(p.ax);
            DoubleBuffer rBuf = NativeBuffers.allocDouble(p.m);
            DoubleBuffer deltaBuf = NativeBuffers.allocDouble(p.n);

            long tFactor0 = System.nanoTime();
            int rank = factorize.apply(id, p.m, p.n, apBuf, aiBuf, axBuf);
            long factorNs = System.nanoTime() - tFactor0;
            assertTrue(rank == p.n,
                    label + " expected full rank, got " + rank + " / " + p.n);

            // 30 rhs vectors; each is the noiseless y with the k-th measurement
            // zeroed (a stand-in for "drop measurement k"). Pattern is fixed so
            // ap/ai/ax stay; only r changes.
            int numRhs = 30;
            long[] perSolve = new long[numRhs];
            double[] residuals = new double[p.m];
            for (int q = 0; q < numRhs; q++) {
                System.arraycopy(p.y, 0, residuals, 0, p.m);
                residuals[q * (p.m / numRhs)] = 0.0;
                NativeBuffers.copyFrom(rBuf, residuals);

                long t0 = System.nanoTime();
                solveCached.apply(id, rBuf, deltaBuf);
                perSolve[q] = System.nanoTime() - t0;
            }

            System.out.printf("%n=== %s [multi-rhs, N=%d, M=%d, K=%d] ===%n",
                    label, p.n, p.m, K_BAND);
            System.out.printf("  factorize once:                     %8.3f ms%n", factorNs / 1e6);
            long total = 0;
            long min = Long.MAX_VALUE;
            long max = 0;
            for (long t : perSolve) {
                total += t;
                if (t < min) {
                    min = t;
                }
                if (t > max) {
                    max = t;
                }
            }
            double mean = (double) total / numRhs;
            System.out.printf("  solveFactorized x%d  mean/min/max:    %8.3f / %.3f / %.3f ms%n",
                    numRhs, mean / 1e6, min / 1e6, max / 1e6);
            System.out.printf("  total (factor + %d solves):          %8.3f ms%n",
                    numRhs, (factorNs + total) / 1e6);
        } finally {
            release.apply(id);
        }
    }

    @Test
    void benchCholeskyMultiRhs() {
        GaussNewtonCholesky s = new GaussNewtonCholesky();
        runMultiRhsBench("Cholesky", s::init, s::factorize, s::solveFactorized, s::release);
    }

    @Test
    void benchKluMultiRhs() {
        GaussNewtonKLU s = new GaussNewtonKLU();
        runMultiRhsBench("KLU", s::init, s::factorize, s::solveFactorized, s::release);
    }

    private static double l2(double[] a, double[] b) {
        double s = 0.0;
        for (int i = 0; i < a.length; i++) {
            double d = a[i] - b[i];
            s += d * d;
        }
        return Math.sqrt(s);
    }

    private static void reportTimings(String label, int n, int m, int k, long[] perIter) {
        System.out.printf("%n=== %s (N=%d, M=%d, K=%d, iters=%d) ===%n",
                label, n, m, k, perIter.length);
        System.out.printf("  iter 0 (cold, pattern build):       %8.3f ms%n", perIter[0] / 1e6);
        if (perIter.length > 1) {
            long warmTotal = 0;
            long warmMin = Long.MAX_VALUE;
            long warmMax = 0;
            for (int i = 1; i < perIter.length; i++) {
                warmTotal += perIter[i];
                if (perIter[i] < warmMin) {
                    warmMin = perIter[i];
                }
                if (perIter[i] > warmMax) {
                    warmMax = perIter[i];
                }
            }
            double warmMean = (double) warmTotal / (perIter.length - 1);
            System.out.printf("  iter 1..%d (warm) mean / min / max:  %8.3f / %.3f / %.3f ms%n",
                    perIter.length - 1, warmMean / 1e6, warmMin / 1e6, warmMax / 1e6);
            System.out.printf("  total across all iters:             %8.3f ms%n",
                    (warmTotal + perIter[0]) / 1e6);
        }
    }
}
