package com.powsybl.mathnative;

import com.powsybl.math.solver.GaussNewtonCholesky;
import com.powsybl.math.solver.GaussNewtonKLU;
import com.powsybl.math.solver.NativeBuffers;
import org.junit.jupiter.api.Test;

import java.nio.DoubleBuffer;
import java.nio.IntBuffer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Verifies the {@code factorize / solveFactorized} pair on both solvers:
 *   - full-rank Ht: factorize returns n, repeated solveFactorized produces the
 *     same answer as the bundled solve() (and as itself across calls).
 *   - rank-deficient Ht (two identical columns of H, so two identical rows of
 *     Ht): factorize returns &lt; n, and solveFactorized throws.
 *   - solveFactorized without a prior factorize: throws.
 */
class GaussNewtonRankTest {

    /**
     * Builds Ht (n x m, CSC) for a least-squares fit of y = a + b*x.
     * H has rows [1, x_i]; Ht has columns [1; x_i].
     * n = 2, m = xData.length, nnz = 2 * m.
     */
    private static Problem linearFit(double[] xData, double[] yData) {
        Problem p = new Problem();
        p.n = 2;
        p.m = xData.length;
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
            ax[idx++] = xData[col];
        }
        ap[p.m] = idx;
        p.ap = NativeBuffers.wrap(ap);
        p.ai = NativeBuffers.wrap(ai);
        p.ax = NativeBuffers.wrap(ax);
        p.r = NativeBuffers.wrap(yData);  // residuals = y - 0*params = y at initial params=(0,0)
        p.sqrtWeights = new double[p.m];
        for (int i = 0; i < p.m; i++) {
            p.sqrtWeights[i] = 1.0;
        }
        return p;
    }

    /**
     * A deliberately rank-deficient Ht: n = 3 but only 2 linearly independent
     * directions exist (column 2 of H equals column 0 of H, so row 2 of Ht
     * equals row 0). Gain matrix C = Ht * Ht' is then n x n with rank 2.
     */
    private static Problem rankDeficient() {
        Problem p = new Problem();
        p.n = 3;
        p.m = 5;
        double[] xData = {0.0, 1.0, 2.0, 3.0, 4.0};
        // Ht columns are H rows [1, x_i, 1] - third entry duplicates the first.
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
            ax[idx++] = xData[col];
            ai[idx] = 2;
            ax[idx++] = 1.0;
        }
        ap[p.m] = idx;
        p.ap = NativeBuffers.wrap(ap);
        p.ai = NativeBuffers.wrap(ai);
        p.ax = NativeBuffers.wrap(ax);
        double[] yData = {1.1, 2.9, 5.2, 6.8, 9.1};
        p.r = NativeBuffers.wrap(yData);
        p.sqrtWeights = new double[p.m];
        for (int i = 0; i < p.m; i++) {
            p.sqrtWeights[i] = 1.0;
        }
        return p;
    }

    private static final class Problem {
        int n;
        int m;
        IntBuffer ap;
        IntBuffer ai;
        DoubleBuffer ax;
        DoubleBuffer r;
        double[] sqrtWeights;
    }

    @Test
    void choleskyFullRankReturnsN() {
        Problem p = linearFit(new double[]{0, 1, 2, 3, 4}, new double[]{1.1, 2.9, 5.2, 6.8, 9.1});
        GaussNewtonCholesky s = new GaussNewtonCholesky();
        s.init("rank-chol-full", p.m, p.n, p.sqrtWeights);
        try {
            int rank = s.factorize("rank-chol-full", p.m, p.n, p.ap, p.ai, p.ax);
            assertEquals(p.n, rank, "full-rank Ht should report rank == n");

            // Repeated solveFactorized with the same r reuses the factor.
            DoubleBuffer delta1 = NativeBuffers.allocDouble(p.n);
            DoubleBuffer delta2 = NativeBuffers.allocDouble(p.n);
            s.solveFactorized("rank-chol-full", p.r, delta1);
            s.solveFactorized("rank-chol-full", p.r, delta2);
            double[] d1 = new double[p.n];
            double[] d2 = new double[p.n];
            NativeBuffers.copyTo(delta1, d1);
            NativeBuffers.copyTo(delta2, d2);
            for (int i = 0; i < p.n; i++) {
                assertEquals(d1[i], d2[i], 0.0,
                        "Re-solving with the same rhs should be bit-identical: i=" + i);
            }
        } finally {
            s.release("rank-chol-full");
        }
    }

    @Test
    void choleskyRankDeficientReturnsLessThanN() {
        Problem p = rankDeficient();
        GaussNewtonCholesky s = new GaussNewtonCholesky();
        s.init("rank-chol-def", p.m, p.n, p.sqrtWeights);
        try {
            int rank = s.factorize("rank-chol-def", p.m, p.n, p.ap, p.ai, p.ax);
            assertNotEquals(p.n, rank, "rank-deficient Ht should not report full rank");
            assertTrue(rank >= 0 && rank < p.n,
                    "deficient rank should be in [0, n); got " + rank);

            DoubleBuffer delta = NativeBuffers.allocDouble(p.n);
            assertThrows(Exception.class,
                    () -> s.solveFactorized("rank-chol-def", p.r, delta),
                    "solveFactorized after rank-deficient factorize must throw");
        } finally {
            s.release("rank-chol-def");
        }
    }

    @Test
    void choleskySolveFactorizedWithoutFactorizeThrows() {
        Problem p = linearFit(new double[]{0, 1, 2, 3, 4}, new double[]{1.1, 2.9, 5.2, 6.8, 9.1});
        GaussNewtonCholesky s = new GaussNewtonCholesky();
        s.init("rank-chol-nofact", p.m, p.n, p.sqrtWeights);
        try {
            DoubleBuffer delta = NativeBuffers.allocDouble(p.n);
            assertThrows(Exception.class,
                    () -> s.solveFactorized("rank-chol-nofact", p.r, delta),
                    "solveFactorized before any factorize must throw");
        } finally {
            s.release("rank-chol-nofact");
        }
    }

    @Test
    void choleskySolveAndFactorizePathsAgree() {
        Problem p = linearFit(new double[]{0, 1, 2, 3, 4}, new double[]{1.1, 2.9, 5.2, 6.8, 9.1});

        double[] viaSolve = new double[p.n];
        {
            GaussNewtonCholesky s = new GaussNewtonCholesky();
            s.init("agree-chol-1", p.m, p.n, p.sqrtWeights);
            try {
                DoubleBuffer delta = NativeBuffers.allocDouble(p.n);
                s.solve("agree-chol-1", p.r, p.m, p.n, p.ap, p.ai, p.ax, delta);
                NativeBuffers.copyTo(delta, viaSolve);
            } finally {
                s.release("agree-chol-1");
            }
        }

        double[] viaPair = new double[p.n];
        {
            GaussNewtonCholesky s = new GaussNewtonCholesky();
            s.init("agree-chol-2", p.m, p.n, p.sqrtWeights);
            try {
                int rank = s.factorize("agree-chol-2", p.m, p.n, p.ap, p.ai, p.ax);
                assertEquals(p.n, rank);
                DoubleBuffer delta = NativeBuffers.allocDouble(p.n);
                s.solveFactorized("agree-chol-2", p.r, delta);
                NativeBuffers.copyTo(delta, viaPair);
            } finally {
                s.release("agree-chol-2");
            }
        }

        for (int i = 0; i < p.n; i++) {
            assertEquals(viaSolve[i], viaPair[i], 1e-12,
                    "solve() and factorize+solveFactorized must agree at index " + i);
        }
    }

    @Test
    void kluFullRankReturnsN() {
        Problem p = linearFit(new double[]{0, 1, 2, 3, 4}, new double[]{1.1, 2.9, 5.2, 6.8, 9.1});
        GaussNewtonKLU s = new GaussNewtonKLU();
        s.init("rank-klu-full", p.m, p.n, p.sqrtWeights);
        try {
            int rank = s.factorize("rank-klu-full", p.m, p.n, p.ap, p.ai, p.ax);
            assertEquals(p.n, rank, "full-rank Ht should report rank == n");

            DoubleBuffer delta1 = NativeBuffers.allocDouble(p.n);
            DoubleBuffer delta2 = NativeBuffers.allocDouble(p.n);
            s.solveFactorized("rank-klu-full", p.r, delta1);
            s.solveFactorized("rank-klu-full", p.r, delta2);
            double[] d1 = new double[p.n];
            double[] d2 = new double[p.n];
            NativeBuffers.copyTo(delta1, d1);
            NativeBuffers.copyTo(delta2, d2);
            for (int i = 0; i < p.n; i++) {
                assertEquals(d1[i], d2[i], 0.0,
                        "Re-solving with the same rhs should be bit-identical: i=" + i);
            }
        } finally {
            s.release("rank-klu-full");
        }
    }

    @Test
    void kluRankDeficientReturnsLessThanN() {
        Problem p = rankDeficient();
        GaussNewtonKLU s = new GaussNewtonKLU();
        s.init("rank-klu-def", p.m, p.n, p.sqrtWeights);
        try {
            int rank = s.factorize("rank-klu-def", p.m, p.n, p.ap, p.ai, p.ax);
            assertNotEquals(p.n, rank, "rank-deficient Ht should not report full rank");
            assertTrue(rank >= 0 && rank < p.n,
                    "deficient rank should be in [0, n); got " + rank);

            DoubleBuffer delta = NativeBuffers.allocDouble(p.n);
            assertThrows(Exception.class,
                    () -> s.solveFactorized("rank-klu-def", p.r, delta),
                    "solveFactorized after rank-deficient factorize must throw");
        } finally {
            s.release("rank-klu-def");
        }
    }

    @Test
    void kluSolveFactorizedWithoutFactorizeThrows() {
        Problem p = linearFit(new double[]{0, 1, 2, 3, 4}, new double[]{1.1, 2.9, 5.2, 6.8, 9.1});
        GaussNewtonKLU s = new GaussNewtonKLU();
        s.init("rank-klu-nofact", p.m, p.n, p.sqrtWeights);
        try {
            DoubleBuffer delta = NativeBuffers.allocDouble(p.n);
            assertThrows(Exception.class,
                    () -> s.solveFactorized("rank-klu-nofact", p.r, delta),
                    "solveFactorized before any factorize must throw");
        } finally {
            s.release("rank-klu-nofact");
        }
    }

    @Test
    void kluSolveAndFactorizePathsAgree() {
        Problem p = linearFit(new double[]{0, 1, 2, 3, 4}, new double[]{1.1, 2.9, 5.2, 6.8, 9.1});

        double[] viaSolve = new double[p.n];
        {
            GaussNewtonKLU s = new GaussNewtonKLU();
            s.init("agree-klu-1", p.m, p.n, p.sqrtWeights);
            try {
                DoubleBuffer delta = NativeBuffers.allocDouble(p.n);
                s.solve("agree-klu-1", p.r, p.m, p.n, p.ap, p.ai, p.ax, delta);
                NativeBuffers.copyTo(delta, viaSolve);
            } finally {
                s.release("agree-klu-1");
            }
        }

        double[] viaPair = new double[p.n];
        {
            GaussNewtonKLU s = new GaussNewtonKLU();
            s.init("agree-klu-2", p.m, p.n, p.sqrtWeights);
            try {
                int rank = s.factorize("agree-klu-2", p.m, p.n, p.ap, p.ai, p.ax);
                assertEquals(p.n, rank);
                DoubleBuffer delta = NativeBuffers.allocDouble(p.n);
                s.solveFactorized("agree-klu-2", p.r, delta);
                NativeBuffers.copyTo(delta, viaPair);
            } finally {
                s.release("agree-klu-2");
            }
        }

        for (int i = 0; i < p.n; i++) {
            assertEquals(viaSolve[i], viaPair[i], 1e-12,
                    "solve() and factorize+solveFactorized must agree at index " + i);
        }
    }

    @Test
    void multipleRhsReuseFactor() {
        // Factor once, solve with several different rhs vectors. Compare each
        // against an independent solve() call. Verifies that the cached factor
        // remains valid across solveFactorized calls.
        double[] xData = {0, 1, 2, 3, 4};
        Problem p = linearFit(xData, new double[]{0, 0, 0, 0, 0});  // r overridden below

        GaussNewtonCholesky s = new GaussNewtonCholesky();
        s.init("multi-rhs", p.m, p.n, p.sqrtWeights);
        try {
            int rank = s.factorize("multi-rhs", p.m, p.n, p.ap, p.ai, p.ax);
            assertEquals(p.n, rank);

            double[][] residuals = {
                {1.1, 2.9, 5.2, 6.8, 9.1},
                {0.0, 1.0, 2.0, 3.0, 4.0},
                {1.0, 0.0, -1.0, 0.0, 1.0},
            };

            for (double[] rArr : residuals) {
                DoubleBuffer rBuf = NativeBuffers.wrap(rArr);
                DoubleBuffer cached = NativeBuffers.allocDouble(p.n);
                s.solveFactorized("multi-rhs", rBuf, cached);
                double[] viaCached = new double[p.n];
                NativeBuffers.copyTo(cached, viaCached);

                GaussNewtonCholesky ref = new GaussNewtonCholesky();
                ref.init("multi-rhs-ref", p.m, p.n, p.sqrtWeights);
                try {
                    DoubleBuffer refOut = NativeBuffers.allocDouble(p.n);
                    ref.solve("multi-rhs-ref", NativeBuffers.wrap(rArr),
                            p.m, p.n, p.ap, p.ai, p.ax, refOut);
                    double[] viaSolve = new double[p.n];
                    NativeBuffers.copyTo(refOut, viaSolve);
                    for (int i = 0; i < p.n; i++) {
                        assertEquals(viaSolve[i], viaCached[i], 1e-12);
                    }
                } finally {
                    ref.release("multi-rhs-ref");
                }
            }
        } finally {
            s.release("multi-rhs");
        }
    }
}
