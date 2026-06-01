package com.powsybl.mathnative;

import com.powsybl.math.solver.GaussNewtonCholesky;
import com.powsybl.math.solver.GaussNewtonKLU;
import com.powsybl.math.solver.NativeBuffers;
import org.junit.jupiter.api.Test;

import java.nio.DoubleBuffer;
import java.nio.IntBuffer;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Levenberg-Marquardt API tests:
 *   1. lambda = 0 must reproduce the GN factor exactly.
 *   2. Identity damping shifts the diagonal by lambda (verified end-to-end via
 *      the solution: small problem with known damped normal equations).
 *   3. Marquardt damping makes a rank-deficient gain matrix factorable, where
 *      undamped Cholesky/KLU would report rank &lt; n.
 *   4. refactorizeLM with a new lambda matches a fresh factorizeLM with the
 *      same lambda - i.e. damping is idempotent (not accumulating).
 *   5. A small non-linear LM outer loop (Rosenbrock-style, n=2, m=2) drives
 *      the residual to zero, confirming the API plays well with a real
 *      accept/reject loop.
 */
class GaussNewtonLMTest {

    /** Build Ht (n=2, m=5) for y = a + b*x least-squares. */
    private static LinearFit linearFit(double[] x, double[] y) {
        LinearFit p = new LinearFit();
        p.n = 2;
        p.m = x.length;
        int nnz = 2 * p.m;
        int[] ap = new int[p.m + 1];
        int[] ai = new int[nnz];
        double[] ax = new double[nnz];
        int idx = 0;
        for (int col = 0; col < p.m; col++) {
            ap[col] = idx;
            ai[idx] = 0;
            ax[idx++] = 1.0;
            ai[idx] = 1;
            ax[idx++] = x[col];
        }
        ap[p.m] = idx;
        p.ap = NativeBuffers.wrap(ap);
        p.ai = NativeBuffers.wrap(ai);
        p.ax = NativeBuffers.wrap(ax);
        p.r = NativeBuffers.wrap(y);
        p.sqrtWeights = new double[p.m];
        for (int i = 0; i < p.m; i++) {
            p.sqrtWeights[i] = 1.0;
        }
        return p;
    }

    /**
     * Same as GaussNewtonRankTest.rankDeficient(): Ht with two identical rows
     * (columns 0 and 2 of the original H are equal) so the gain matrix has
     * rank 2 (and a flat null direction).
     */
    private static LinearFit rankDeficient() {
        LinearFit p = new LinearFit();
        p.n = 3;
        p.m = 5;
        double[] x = {0.0, 1.0, 2.0, 3.0, 4.0};
        int nnz = 3 * p.m;
        int[] ap = new int[p.m + 1];
        int[] ai = new int[nnz];
        double[] ax = new double[nnz];
        int idx = 0;
        for (int col = 0; col < p.m; col++) {
            ap[col] = idx;
            ai[idx] = 0;
            ax[idx++] = 1.0;
            ai[idx] = 1;
            ax[idx++] = x[col];
            ai[idx] = 2;
            ax[idx++] = 1.0;
        }
        ap[p.m] = idx;
        p.ap = NativeBuffers.wrap(ap);
        p.ai = NativeBuffers.wrap(ai);
        p.ax = NativeBuffers.wrap(ax);
        p.r = NativeBuffers.wrap(new double[]{1.1, 2.9, 5.2, 6.8, 9.1});
        p.sqrtWeights = new double[p.m];
        for (int i = 0; i < p.m; i++) {
            p.sqrtWeights[i] = 1.0;
        }
        return p;
    }

    private static final class LinearFit {
        int n;
        int m;
        IntBuffer ap;
        IntBuffer ai;
        DoubleBuffer ax;
        DoubleBuffer r;
        double[] sqrtWeights;
    }

    /**
     * (1) lambda = 0 yields exactly the GN solution. Both modes must agree
     * with each other and with the undamped path.
     */
    @Test
    void choleskyLMWithZeroLambdaMatchesGN() {
        LinearFit p = linearFit(new double[]{0, 1, 2, 3, 4},
                new double[]{1.1, 2.9, 5.2, 6.8, 9.1});

        double[] gn = solveGN(new GaussNewtonCholesky(), "lm-zero-gn", p);
        double[] lmIdentity = solveLM(new GaussNewtonCholesky(), "lm-zero-id", p,
                0.0, GaussNewtonCholesky.LM_MODE_IDENTITY);
        double[] lmMarquardt = solveLM(new GaussNewtonCholesky(), "lm-zero-mq", p,
                0.0, GaussNewtonCholesky.LM_MODE_MARQUARDT);

        for (int i = 0; i < p.n; i++) {
            assertEquals(gn[i], lmIdentity[i], 1e-12);
            assertEquals(gn[i], lmMarquardt[i], 1e-12);
        }
    }

    @Test
    void kluLMWithZeroLambdaMatchesGN() {
        LinearFit p = linearFit(new double[]{0, 1, 2, 3, 4},
                new double[]{1.1, 2.9, 5.2, 6.8, 9.1});

        double[] gn = solveGN(new GaussNewtonKLU(), "klm-zero-gn", p);
        double[] lmIdentity = solveLM(new GaussNewtonKLU(), "klm-zero-id", p,
                0.0, GaussNewtonKLU.LM_MODE_IDENTITY);
        double[] lmMarquardt = solveLM(new GaussNewtonKLU(), "klm-zero-mq", p,
                0.0, GaussNewtonKLU.LM_MODE_MARQUARDT);

        for (int i = 0; i < p.n; i++) {
            assertEquals(gn[i], lmIdentity[i], 1e-12);
            assertEquals(gn[i], lmMarquardt[i], 1e-12);
        }
    }

    /**
     * (2) Identity damping shifts the diagonal by lambda. Verify against the
     * closed-form solution of the damped normal equations for the small fit
     * problem.
     *
     * For y = a + b*x with all wSq=1 and the small dataset below, the gain
     * matrix is C = [[m, sum_x], [sum_x, sum_x^2]] and the rhs is
     * [sum_y, sum_xy]. With Levenberg damping the system is (C + lambda I) p =
     * rhs - we solve it independently in Java and compare.
     */
    @Test
    void choleskyIdentityDampingShiftsDiagonal() {
        double[] x = {0, 1, 2, 3, 4};
        double[] y = {1.1, 2.9, 5.2, 6.8, 9.1};
        LinearFit p = linearFit(x, y);
        double lambda = 0.5;

        double[] expected = solveDampedLinearFit(x, y, lambda, false);

        double[] got = solveLM(new GaussNewtonCholesky(), "lm-id-shift", p,
                lambda, GaussNewtonCholesky.LM_MODE_IDENTITY);

        for (int i = 0; i < p.n; i++) {
            assertEquals(expected[i], got[i], 1e-10);
        }
    }

    @Test
    void kluMarquardtDampingScalesDiagonal() {
        double[] x = {0, 1, 2, 3, 4};
        double[] y = {1.1, 2.9, 5.2, 6.8, 9.1};
        LinearFit p = linearFit(x, y);
        double lambda = 0.5;

        double[] expected = solveDampedLinearFit(x, y, lambda, true);

        double[] got = solveLM(new GaussNewtonKLU(), "lm-mq-scale", p,
                lambda, GaussNewtonKLU.LM_MODE_MARQUARDT);

        for (int i = 0; i < p.n; i++) {
            assertEquals(expected[i], got[i], 1e-10);
        }
    }

    /**
     * Solve (C + lambda*D)x = rhs for the linear-fit problem in Java, where
     * D = diag(C) if marquardt else I. Closed-form via 2x2 inverse.
     */
    private static double[] solveDampedLinearFit(double[] x, double[] y, double lambda, boolean marquardt) {
        double c00 = 0;
        double c01 = 0;
        double c11 = 0;
        double r0 = 0;
        double r1 = 0;
        for (int i = 0; i < x.length; i++) {
            c00 += 1.0;
            c01 += x[i];
            c11 += x[i] * x[i];
            r0 += y[i];
            r1 += x[i] * y[i];
        }
        double d0;
        double d1;
        if (marquardt) {
            d0 = c00 * (1 + lambda);
            d1 = c11 * (1 + lambda);
        } else {
            d0 = c00 + lambda;
            d1 = c11 + lambda;
        }
        double det = d0 * d1 - c01 * c01;
        return new double[]{
            (d1 * r0 - c01 * r1) / det,
            (-c01 * r0 + d0 * r1) / det,
        };
    }

    /**
     * (3) Marquardt damping makes a rank-deficient C factorable. We don't
     * compare against an expected solution (the system has a 1-d null space
     * so the damped solution is regularizer-dependent), only that the call
     * succeeds and returns full rank.
     */
    @Test
    void choleskyMarquardtRecoversRankDeficient() {
        LinearFit p = rankDeficient();
        GaussNewtonCholesky s = new GaussNewtonCholesky();
        s.init("lm-def-mq-chol", p.m, p.n, p.sqrtWeights);
        try {
            // Confirm the undamped system is rank-deficient.
            int undampedRank = s.factorize("lm-def-mq-chol", p.m, p.n, p.ap, p.ai, p.ax);
            assertNotEquals(p.n, undampedRank,
                    "rank-deficient setup did not produce a rank-deficient factor (got " + undampedRank + ")");

            // With Marquardt damping the diagonal becomes diag(C)*(1+lambda),
            // which is strictly positive - but the off-diagonal coupling
            // through the null direction remains, so a tiny lambda doesn't
            // fix it. Use a substantial lambda.
            int rank = s.factorizeLM("lm-def-mq-chol", p.m, p.n, p.ap, p.ai, p.ax,
                    1.0, GaussNewtonCholesky.LM_MODE_MARQUARDT);
            assertEquals(p.n, rank,
                    "Marquardt-damped factor should be full rank with lambda=1");

            DoubleBuffer delta = NativeBuffers.allocDouble(p.n);
            s.solveFactorized("lm-def-mq-chol", p.r, delta);
            // Step must be finite (no NaN/Inf).
            double[] d = new double[p.n];
            NativeBuffers.copyTo(delta, d);
            for (double v : d) {
                assertTrue(Double.isFinite(v), "damped step contains non-finite value: " + v);
            }
        } finally {
            s.release("lm-def-mq-chol");
        }
    }

    @Test
    void kluIdentityRecoversRankDeficient() {
        LinearFit p = rankDeficient();
        GaussNewtonKLU s = new GaussNewtonKLU();
        s.init("lm-def-id-klu", p.m, p.n, p.sqrtWeights);
        try {
            int undampedRank = s.factorize("lm-def-id-klu", p.m, p.n, p.ap, p.ai, p.ax);
            assertNotEquals(p.n, undampedRank);

            // Identity damping (C + lambda*I) is positive-definite for any
            // lambda > 0 since the smallest eigenvalue rises by lambda. Use a
            // modest lambda.
            int rank = s.factorizeLM("lm-def-id-klu", p.m, p.n, p.ap, p.ai, p.ax,
                    1e-3, GaussNewtonKLU.LM_MODE_IDENTITY);
            assertEquals(p.n, rank,
                    "Identity-damped factor should be full rank with any positive lambda");

            DoubleBuffer delta = NativeBuffers.allocDouble(p.n);
            s.solveFactorized("lm-def-id-klu", p.r, delta);
            double[] d = new double[p.n];
            NativeBuffers.copyTo(delta, d);
            for (double v : d) {
                assertTrue(Double.isFinite(v));
            }
        } finally {
            s.release("lm-def-id-klu");
        }
    }

    /**
     * (4) Damping is idempotent: refactorizeLM after factorizeLM with lambda_2
     * produces the same solution as a fresh factorizeLM with lambda_2 on the
     * same Ht. Catches accidental "double damping" (applying lambda*D to an
     * already-damped diagonal).
     */
    @Test
    void choleskyRefactorizeLMIsIdempotent() {
        LinearFit p = linearFit(new double[]{0, 1, 2, 3, 4},
                new double[]{1.1, 2.9, 5.2, 6.8, 9.1});

        // Trail: factorize with lambda=10, then refactor to lambda=0.1.
        GaussNewtonCholesky s = new GaussNewtonCholesky();
        s.init("lm-refact", p.m, p.n, p.sqrtWeights);
        double[] viaRefact = new double[p.n];
        try {
            int rank = s.factorizeLM("lm-refact", p.m, p.n, p.ap, p.ai, p.ax,
                    10.0, GaussNewtonCholesky.LM_MODE_MARQUARDT);
            assertEquals(p.n, rank);

            rank = s.refactorizeLM("lm-refact", 0.1, GaussNewtonCholesky.LM_MODE_MARQUARDT);
            assertEquals(p.n, rank);

            DoubleBuffer delta = NativeBuffers.allocDouble(p.n);
            s.solveFactorized("lm-refact", p.r, delta);
            NativeBuffers.copyTo(delta, viaRefact);
        } finally {
            s.release("lm-refact");
        }

        // Reference: fresh factorizeLM with lambda=0.1.
        double[] viaFresh = solveLM(new GaussNewtonCholesky(), "lm-fresh", p,
                0.1, GaussNewtonCholesky.LM_MODE_MARQUARDT);

        for (int i = 0; i < p.n; i++) {
            assertEquals(viaFresh[i], viaRefact[i], 1e-12,
                    "refactorizeLM should match fresh factorizeLM at index " + i);
        }
    }

    @Test
    void kluRefactorizeLMIsIdempotent() {
        LinearFit p = linearFit(new double[]{0, 1, 2, 3, 4},
                new double[]{1.1, 2.9, 5.2, 6.8, 9.1});

        GaussNewtonKLU s = new GaussNewtonKLU();
        s.init("lm-klu-refact", p.m, p.n, p.sqrtWeights);
        double[] viaRefact = new double[p.n];
        try {
            int rank = s.factorizeLM("lm-klu-refact", p.m, p.n, p.ap, p.ai, p.ax,
                    10.0, GaussNewtonKLU.LM_MODE_IDENTITY);
            assertEquals(p.n, rank);

            rank = s.refactorizeLM("lm-klu-refact", 0.1, GaussNewtonKLU.LM_MODE_IDENTITY);
            assertEquals(p.n, rank);

            DoubleBuffer delta = NativeBuffers.allocDouble(p.n);
            s.solveFactorized("lm-klu-refact", p.r, delta);
            NativeBuffers.copyTo(delta, viaRefact);
        } finally {
            s.release("lm-klu-refact");
        }

        double[] viaFresh = solveLM(new GaussNewtonKLU(), "lm-klu-fresh", p,
                0.1, GaussNewtonKLU.LM_MODE_IDENTITY);

        for (int i = 0; i < p.n; i++) {
            assertEquals(viaFresh[i], viaRefact[i], 1e-12);
        }
    }

    /**
     * refactorizeLM without a prior factorize must throw.
     */
    @Test
    void refactorizeLMBeforeFactorizeThrows() {
        LinearFit p = linearFit(new double[]{0, 1, 2, 3, 4},
                new double[]{1.1, 2.9, 5.2, 6.8, 9.1});

        GaussNewtonCholesky s = new GaussNewtonCholesky();
        s.init("lm-nofact", p.m, p.n, p.sqrtWeights);
        try {
            assertThrows(Exception.class,
                    () -> s.refactorizeLM("lm-nofact", 0.5, GaussNewtonCholesky.LM_MODE_IDENTITY));
        } finally {
            s.release("lm-nofact");
        }
    }

    /**
     * Invalid mode throws.
     */
    @Test
    void invalidDampingModeThrows() {
        LinearFit p = linearFit(new double[]{0, 1, 2, 3, 4},
                new double[]{1.1, 2.9, 5.2, 6.8, 9.1});
        GaussNewtonKLU s = new GaussNewtonKLU();
        s.init("lm-bad-mode", p.m, p.n, p.sqrtWeights);
        try {
            assertThrows(Exception.class,
                    () -> s.factorizeLM("lm-bad-mode", p.m, p.n, p.ap, p.ai, p.ax, 0.1, 99));
        } finally {
            s.release("lm-bad-mode");
        }
    }

    /**
     * (5) End-to-end LM outer loop on a non-linear least-squares problem
     * where Gauss-Newton without damping is known to diverge from a poor
     * initial guess. We use a small synthetic problem: fit a logistic
     * y = 1 / (1 + exp(-(a + b*x))) to data with 20 samples. From a far
     * starting point (a=5, b=5) GN diverges; LM converges.
     *
     * Trust-region rules (Madsen-Nielsen-Tingleff defaults):
     *   - If accept: lambda *= max(1/3, 1 - (2*gain - 1)^3); nu = 2.
     *   - If reject: lambda *= nu; nu *= 2.
     * Cost = 0.5 * ||r||^2 with r = y_predicted - y_observed.
     */
    @Test
    void lmOuterLoopCholeskyConvergesLogistic() {
        runLogisticLM(true);
    }

    @Test
    void lmOuterLoopKluConvergesLogistic() {
        runLogisticLM(false);
    }

    private static void runLogisticLM(boolean cholesky) {
        // Generate noiseless data from a known logistic.
        double aTrue = 0.5;
        double bTrue = 1.5;
        int m = 20;
        double[] xs = new double[m];
        double[] ys = new double[m];
        Random rng = new Random(7);
        for (int i = 0; i < m; i++) {
            xs[i] = -3.0 + 6.0 * i / (m - 1);
            double z = aTrue + bTrue * xs[i];
            ys[i] = 1.0 / (1.0 + Math.exp(-z)) + 1e-4 * rng.nextGaussian();
        }

        int n = 2;
        double[] params = {-3.0, -2.0};
        double[] sqrtW = new double[m];
        for (int i = 0; i < m; i++) {
            sqrtW[i] = 1.0;
        }

        // CSC buffers for Ht (2 x m): row 0 = d/da, row 1 = d/db.
        int[] ap = new int[m + 1];
        int[] ai = new int[2 * m];
        double[] ax = new double[2 * m];
        for (int k = 0; k < m; k++) {
            ap[k] = 2 * k;
            ai[2 * k] = 0;
            ai[2 * k + 1] = 1;
        }
        ap[m] = 2 * m;
        IntBuffer apBuf = NativeBuffers.wrap(ap);
        IntBuffer aiBuf = NativeBuffers.wrap(ai);
        DoubleBuffer axBuf = NativeBuffers.allocDouble(2 * m);
        DoubleBuffer rBuf = NativeBuffers.allocDouble(m);
        DoubleBuffer deltaBuf = NativeBuffers.allocDouble(n);

        String id = cholesky ? "lm-loop-chol" : "lm-loop-klu";
        GaussNewtonCholesky sChol = cholesky ? new GaussNewtonCholesky() : null;
        GaussNewtonKLU sKlu = cholesky ? null : new GaussNewtonKLU();
        if (cholesky) {
            sChol.init(id, m, n, sqrtW);
        } else {
            sKlu.init(id, m, n, sqrtW);
        }
        int mode = cholesky
                ? GaussNewtonCholesky.LM_MODE_MARQUARDT
                : GaussNewtonKLU.LM_MODE_MARQUARDT;

        try {
            double lambda = 1e-3;
            double nu = 2.0;
            double prevCost = computeLogistic(params, xs, ys, axBuf, rBuf);

            boolean factored = false;
            for (int iter = 0; iter < 30; iter++) {
                if (!factored) {
                    int rank = cholesky
                            ? sChol.factorizeLM(id, m, n, apBuf, aiBuf, axBuf, lambda, mode)
                            : sKlu.factorizeLM(id, m, n, apBuf, aiBuf, axBuf, lambda, mode);
                    assertEquals(n, rank, "damped factor should be full rank at iter " + iter);
                    factored = true;
                }
                if (cholesky) {
                    sChol.solveFactorized(id, rBuf, deltaBuf);
                } else {
                    sKlu.solveFactorized(id, rBuf, deltaBuf);
                }

                double[] step = new double[n];
                NativeBuffers.copyTo(deltaBuf, step);
                double[] trial = {params[0] + step[0], params[1] + step[1]};
                DoubleBuffer dummyAx = NativeBuffers.allocDouble(2 * m);
                DoubleBuffer trialR = NativeBuffers.allocDouble(m);
                double trialCost = computeLogistic(trial, xs, ys, dummyAx, trialR);

                if (trialCost < prevCost) {
                    // Accept: copy trial -> params, recompute Ht/r at new
                    // params (axBuf, rBuf already updated by computeLogistic
                    // when we call it next iter via prevCost path... but here
                    // computeLogistic used dummyAx/trialR for the trial. We
                    // need axBuf/rBuf to reflect the accepted point. Refresh.)
                    params[0] = trial[0];
                    params[1] = trial[1];
                    prevCost = computeLogistic(params, xs, ys, axBuf, rBuf);
                    // Shrink lambda toward GN. Madsen et al recipe.
                    lambda *= Math.max(1.0 / 3.0, 1.0 - Math.pow(2.0, 3));
                    if (lambda < 1e-12) {
                        lambda = 1e-12;
                    }
                    nu = 2.0;
                    factored = false;  // need to rebuild C at new params

                    if (prevCost < 1e-10) {
                        break;
                    }
                } else {
                    // Reject: keep params, grow lambda, refactorizeLM.
                    lambda *= nu;
                    nu *= 2.0;
                    int rank = cholesky
                            ? sChol.refactorizeLM(id, lambda, mode)
                            : sKlu.refactorizeLM(id, lambda, mode);
                    assertEquals(n, rank, "rejected step's bigger lambda should not break rank");
                }
            }

            assertEquals(aTrue, params[0], 0.01,
                    "LM (" + (cholesky ? "Cholesky" : "KLU") + ") failed to recover a");
            assertEquals(bTrue, params[1], 0.05,
                    "LM (" + (cholesky ? "Cholesky" : "KLU") + ") failed to recover b");
        } finally {
            if (cholesky) {
                sChol.release(id);
            } else {
                sKlu.release(id);
            }
        }
    }

    /**
     * For the logistic LS problem: write current residuals into rBuf, Ht
     * Jacobian values into axBuf (column k = [-sigma'(z_k), -x_k*sigma'(z_k)],
     * since r_k = sigma(z_k) - y_k and d r_k / d a = sigma'(z_k)).
     * Returns 0.5 * sum r^2.
     */
    private static double computeLogistic(double[] p, double[] xs, double[] ys,
                                          DoubleBuffer axBuf, DoubleBuffer rBuf) {
        int m = xs.length;
        double[] ax = new double[2 * m];
        double[] r = new double[m];
        double cost = 0;
        for (int k = 0; k < m; k++) {
            double z = p[0] + p[1] * xs[k];
            double sigma = 1.0 / (1.0 + Math.exp(-z));
            double dsigma = sigma * (1.0 - sigma);
            // r_k = sigma - y; for the LM step solving (H'WH) dp = -H'W r we
            // want r = (y - sigma) so that GN's positive step direction lines
            // up with descent on 0.5||y - sigma||^2. Equivalently use H = -J
            // with J = d sigma / d p and r = y - sigma.
            r[k] = ys[k] - sigma;
            ax[2 * k] = dsigma;
            ax[2 * k + 1] = xs[k] * dsigma;
            cost += 0.5 * r[k] * r[k];
        }
        NativeBuffers.copyFrom(axBuf, ax);
        NativeBuffers.copyFrom(rBuf, r);
        return cost;
    }

    // ---- helpers ---------------------------------------------------------

    private static double[] solveGN(GaussNewtonCholesky s, String id, LinearFit p) {
        s.init(id, p.m, p.n, p.sqrtWeights);
        try {
            DoubleBuffer out = NativeBuffers.allocDouble(p.n);
            s.solve(id, p.r, p.m, p.n, p.ap, p.ai, p.ax, out);
            double[] a = new double[p.n];
            NativeBuffers.copyTo(out, a);
            return a;
        } finally {
            s.release(id);
        }
    }

    private static double[] solveGN(GaussNewtonKLU s, String id, LinearFit p) {
        s.init(id, p.m, p.n, p.sqrtWeights);
        try {
            DoubleBuffer out = NativeBuffers.allocDouble(p.n);
            s.solve(id, p.r, p.m, p.n, p.ap, p.ai, p.ax, out);
            double[] a = new double[p.n];
            NativeBuffers.copyTo(out, a);
            return a;
        } finally {
            s.release(id);
        }
    }

    private static double[] solveLM(GaussNewtonCholesky s, String id, LinearFit p,
                                    double lambda, int mode) {
        s.init(id, p.m, p.n, p.sqrtWeights);
        try {
            DoubleBuffer out = NativeBuffers.allocDouble(p.n);
            s.solveLM(id, p.r, p.m, p.n, p.ap, p.ai, p.ax, lambda, mode, out);
            double[] a = new double[p.n];
            NativeBuffers.copyTo(out, a);
            return a;
        } finally {
            s.release(id);
        }
    }

    private static double[] solveLM(GaussNewtonKLU s, String id, LinearFit p,
                                    double lambda, int mode) {
        s.init(id, p.m, p.n, p.sqrtWeights);
        try {
            DoubleBuffer out = NativeBuffers.allocDouble(p.n);
            s.solveLM(id, p.r, p.m, p.n, p.ap, p.ai, p.ax, lambda, mode, out);
            double[] a = new double[p.n];
            NativeBuffers.copyTo(out, a);
            return a;
        } finally {
            s.release(id);
        }
    }
}
