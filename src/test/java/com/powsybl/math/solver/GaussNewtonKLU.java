package com.powsybl.math.solver;

import com.powsybl.math.AbstractMathNative;

import java.nio.DoubleBuffer;
import java.nio.IntBuffer;

/**
 * Sparse LU decomposition using KLU for weighted least squares problems.
 * Used in Gauss-Newton algorithm to solve normal equations: (H'WH)Δx = H'Wr
 */
public class GaussNewtonKLU extends AbstractMathNative {

    /**
     * Initialize the decomposition with the normal equations matrix structure.
     * @param m number of rows in Jacobian
     */
    public native void init(String id, int m, int n, double[] wDiag);

    /**
     * Assemble C = H'WH from Ht and factorize it. Caches the factor and the Ht
     * values so subsequent {@link #solveFactorized} calls can reuse them with
     * different residual vectors - useful for observability checks (factor once,
     * inspect rank) and bad-data analysis (re-solve with adjusted residuals).
     *
     * @return n if the factorization succeeded (system is full-rank per the
     *         solver's pivot heuristic), or the column index k &lt; n where
     *         rank-deficiency was first detected (KLU's common.singular_col).
     *         Heuristic: a well-scaled near-singular matrix can still report n,
     *         and a poorly-scaled full-rank matrix can report less. For
     *         guaranteed rank, use pivoted QR or SVD.
     */
    public native int factorize(String id, int m, int n,
                                IntBuffer ap, IntBuffer ai, DoubleBuffer ax);

    /**
     * Solve C Δx = H'W r using the factor cached by the most recent
     * {@link #factorize} call. May be called multiple times with different r.
     * Throws if no factor is cached or if the last factorize was rank-deficient.
     */
    public native void solveFactorized(String id, DoubleBuffer r, DoubleBuffer result);

    /**
     * Solve C x = b directly using the factor cached by the most recent
     * {@link #factorize} call. Unlike {@link #solveFactorized}, the right-hand
     * side is taken as-is from the {@code b} buffer (length n) and no
     * H'W r assembly is performed. Useful for computing individual columns of
     * {@code C^-1} - e.g. state covariance diagonals via b = e_j.
     * <p>
     * Throws if no factor is cached or if the last factorize was rank-deficient.
     */
    public native void solveFactorizedRaw(String id, DoubleBuffer b, DoubleBuffer result);

    /**
     * Convenience: factorize + solveFactorized in one call. Throws if the
     * factorization is rank-deficient. Equivalent to:
     * <pre>
     *   if (factorize(id, m, n, ap, ai, ax) != n) throw ...;
     *   solveFactorized(id, r, result);
     * </pre>
     *
     * All buffer arguments MUST be direct (ByteBuffer.allocateDirect + native order).
     *
     * @param r residual vector (size m)
     * @param m number of rows in Jacobian
     * @param n number of columns in Jacobian
     * @param ap column pointers for transpose of Jacobian Ht (size m+1)
     * @param ai row indices for Ht (size nnz)
     * @param ax values for Ht (size nnz)
     * @param result output for delta_x (size n)
     */
    public native void solve(String id, DoubleBuffer r, int m, int n,
                             IntBuffer ap, IntBuffer ai, DoubleBuffer ax, DoubleBuffer result);

    /** LM damping mode for {@link #factorizeLM}, {@link #refactorizeLM}, {@link #solveLM}. */
    public static final int LM_MODE_IDENTITY = 0;
    /** LM damping mode for {@link #factorizeLM}, {@link #refactorizeLM}, {@link #solveLM}. */
    public static final int LM_MODE_MARQUARDT = 1;

    /**
     * Levenberg-Marquardt factorize: assemble C = H'WH, add the LM damping
     * lambda*D to its diagonal, then factorize. Caches the factor and Ht
     * values so subsequent {@link #solveFactorized} or {@link #refactorizeLM}
     * calls can reuse them. With lambda == 0 this is equivalent to
     * {@link #factorize}.
     *
     * @param mode {@link #LM_MODE_IDENTITY} (D = I, classic Levenberg) or
     *             {@link #LM_MODE_MARQUARDT} (D = diag(H'WH), scale-invariant).
     * @return n if the damped factor is full rank, otherwise the column where
     *         rank deficiency was detected (KLU's common.singular_col).
     */
    public native int factorizeLM(String id, int m, int n,
                                  IntBuffer ap, IntBuffer ai, DoubleBuffer ax,
                                  double lambda, int mode);

    /**
     * Re-factorize the cached gain matrix with a new damping. Useful in the
     * LM outer loop after a step is rejected: the Ht and contribution map are
     * unchanged, so we just rewrite the diagonals and factor again - no need
     * for the caller to pass Ht. Requires a prior {@link #factorize} or
     * {@link #factorizeLM} call to populate the pattern.
     *
     * @return same convention as {@link #factorizeLM}.
     */
    public native int refactorizeLM(String id, double lambda, int mode);

    /**
     * Convenience: factorizeLM + solveFactorized in one call. Throws if the
     * damped factor is still rank-deficient (typically a signal to increase
     * lambda).
     */
    public native void solveLM(String id, DoubleBuffer r, int m, int n,
                               IntBuffer ap, IntBuffer ai, DoubleBuffer ax,
                               double lambda, int mode, DoubleBuffer result);

    /**
     * Release all native resources.
     */
    public native void release(String id);

}
