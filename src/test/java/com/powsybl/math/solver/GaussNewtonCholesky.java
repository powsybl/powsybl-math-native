package com.powsybl.math.solver;

import com.powsybl.math.AbstractMathNative;

/**
 * Sparse Cholesky decomposition using CHOLMOD for weighted least squares problems.
 * Used in Gauss-Newton algorithm to solve normal equations: (H'WH)Δx = H'Wr
 */
public class GaussNewtonCholesky extends AbstractMathNative {

    /**
     * Initialize the decomposition with the normal equations matrix structure.
     * @param m number of rows in Jacobian
     */
    public native void init(String id, int m, int n, double[] wDiag);

    /**
     * Solve the weighted normal equations: (H'WH)Δx = H'Wr
     * @param wDiag sqrt of weight diagonal vector
     * @param r residual vector
     * @param m number of rows in Jacobian
     * @param n number of columns in Jacobian
     * @param ap column pointers for transpose of Jacobian (Ht)
     * @param ai row indices for transpose of Jacobian (Ht)
     * @param ax values for transpose of Jacobian (Ht)
     * @param result output array for delta_x (must be pre-allocated with size n)
     */
    public native void solve(String id, double[] wDiag, double[] r, int m, int n,
                             int[] ap, int[] ai, double[] ax, double[] result);

    /**
     * Release all native resources.
     */
    public native void release(String id);

}