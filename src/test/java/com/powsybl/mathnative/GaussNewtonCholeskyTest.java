package com.powsybl.mathnative;

import com.powsybl.math.solver.GaussNewtonCholesky;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Test for Gauss-Newton least squares minimization using CHOLMOD.
 *
 * Problem: Fit a simple exponential model y = a * exp(b * x) to noisy data
 * We'll linearize it and use Gauss-Newton iterations.
 *
 * @author Geoffroy Jamgotchian {@literal <geoffroy.jamgotchian at rte-france.com>}
 */
class GaussNewtonCholeskyTest {

    private static void printSparseMatrix(String name, int m, int n, int[] ap, int[] ai, double[] ax) {
        System.out.println("\n=== " + name + " (" + m + "x" + n + ") ===");

        // Print CSC format details
        System.out.println("CSC Format:");
        System.out.print("ap = [");
        for (int i = 0; i < ap.length; i++) {
            System.out.print(ap[i] + (i < ap.length - 1 ? ", " : ""));
        }
        System.out.println("]");

        System.out.print("ai = [");
        for (int i = 0; i < ai.length; i++) {
            System.out.print(ai[i] + (i < ai.length - 1 ? ", " : ""));
        }
        System.out.println("]");

        System.out.print("ax = [");
        for (int i = 0; i < ax.length; i++) {
            System.out.printf("%.3f", ax[i]);
            if (i < ax.length - 1) System.out.print(", ");
        }
        System.out.println("]");

        // Print dense representation
        System.out.println("\nDense representation:");
        double[][] dense = new double[m][n];

        // Convert CSC to dense
        for (int col = 0; col < n; col++) {
            for (int idx = ap[col]; idx < ap[col + 1]; idx++) {
                int row = ai[idx];
                double value = ax[idx];
                dense[row][col] = value;
            }
        }

        // Print dense matrix
        for (int row = 0; row < m; row++) {
            System.out.print("  [");
            for (int col = 0; col < n; col++) {
                System.out.printf("%8.3f", dense[row][col]);
                if (col < n - 1) System.out.print(", ");
            }
            System.out.println("]");
        }
        System.out.println();
    }

    /**
     * Simple test: fit a line y = a + b*x to data points
     * This is a linear problem so it should converge in 1 iteration.
     */
    @Test
    void testLinearLeastSquares() {
        // Data points: (x, y) with some noise
        double[] xData = {0.0, 1.0, 2.0, 3.0, 4.0};
        double[] yData = {1.1, 2.9, 5.2, 6.8, 9.1}; // approximately y = 1 + 2*x with noise
        int m = xData.length; // number of observations
        int n = 2; // number of parameters (a, b)

        // Initial guess
        double[] params = {1.0, 1.0}; // start with a=0, b=0

        // Weights (equal weights for simplicity)
        double[] weights = new double[m];
        double[] sqrtWeights = new double[m];
        for (int i = 0; i < m; i++) {
            weights[i] = 1.0;
            sqrtWeights[i] = Math.sqrt(weights[i]);
        }

        GaussNewtonCholesky cholesky = new GaussNewtonCholesky();
        cholesky.init("test", m, n, sqrtWeights);

        try {
            // Gauss-Newton iterations
            int maxIter = 10;
            double tolerance = 1e-6;

            for (int iter = 0; iter < maxIter; iter++) {
                // 1. Compute residuals: r = y - f(x, params)
                double[] residuals = new double[m];
                for (int i = 0; i < m; i++) {
                    double predicted = params[0] + params[1] * xData[i];
                    residuals[i] = yData[i] - predicted;
                }

                // 2. Compute Jacobian H: H[i][j] = -∂f/∂params[j]
                // For y = a + b*x: ∂f/∂a = 1, ∂f/∂b = x
                // So H = -[1, x[0]; 1, x[1]; 1, x[2]; ...]
                // We need Ht (transpose) in CSC format

                // Build Ht (2 x m) in CSC format
                // Column 0: all -1's (derivative w.r.t. a)
                // Column 1: -x values (derivative w.r.t. b)
                int[] ap = new int[m + 1];
                int[] ai = new int[m * n];
                double[] ax = new double[m * n];

                int idx = 0;
                for (int col = 0; col < m; col++) {
                    ap[col] = idx;
                    // Each column has 2 entries (one for each parameter)
                    ai[idx] = 0; // parameter a
                    ax[idx] = 1.0;
                    idx++;
                    ai[idx] = 1; // parameter b
                    ax[idx] = xData[col];
                    idx++;
                }
                ap[m] = idx;

//                printSparseMatrix("Ht (Jacobian transpose)", n, m, ap, ai, ax);

                // 4. Solve for delta: (H'WH)Δx = H'Wr
                double[] delta = new double[n];
                cholesky.solve("test", sqrtWeights, residuals, m, n, ap, ai, ax, delta);

                // 5. Update parameters
                for (int i = 0; i < n; i++) {
                    params[i] += delta[i];
                }

                // 6. Check convergence
                double deltaNorm = 0.0;
                for (double d : delta) {
                    deltaNorm += d * d;
                }
                deltaNorm = Math.sqrt(deltaNorm);

                System.out.printf("Iteration %d: params = [%.6f, %.6f], ||delta|| = %.9f%n",
                        iter, params[0], params[1], deltaNorm);

                if (deltaNorm < tolerance) {
                    System.out.println("Converged!");
                    break;
                }
            }

            // Expected result: approximately a=1.0, b=2.0
            assertArrayEquals(new double[]{1.02, 2.0}, params, 0.1,
                    "Parameters should fit the line y = 1 + 2*x");

        } finally {
            cholesky.release("test");
        }
    }

    /**
     * More complex test: fit an exponential decay model
     * y = a * exp(-b * x) to data
     */
    @Test
    void testNonlinearLeastSquares() {
        // Data points: true model is y = 5 * exp(-0.5 * x)
        double[] xData = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
        double[] yData = {5.0, 3.03, 1.84, 1.12, 0.68, 0.41};
        int m = xData.length;
        int n = 2; // parameters: a, b

        // Initial guess (important for nonlinear problems)
        double[] params = {1.0, -0.1}; // start near true values

        // Equal weights
        double[] sqrtWeights = new double[m];
        for (int i = 0; i < m; i++) {
            sqrtWeights[i] = 1.0;
        }

        GaussNewtonCholesky cholesky = new GaussNewtonCholesky();
        cholesky.init("test", m, n, sqrtWeights);

        try {
            int maxIter = 20;
            double tolerance = 1e-6;

            for (int iter = 0; iter < maxIter; iter++) {
                // 1. Compute residuals
                double[] residuals = new double[m];
                for (int i = 0; i < m; i++) {
                    double predicted = params[0] * Math.exp(params[1] * xData[i]);
                    residuals[i] = yData[i] - predicted;
                }

                // 2. Compute Jacobian: H[i][j] = -∂f/∂params[j]
                // For f = a * exp(-b * x):
                //   ∂f/∂a = exp(-b * x)
                //   ∂f/∂b = -a * x * exp(-b * x)

                int[] ap = new int[m + 1];
                int[] ai = new int[m * n];
                double[] ax = new double[m * n];

                int idx = 0;
                for (int col = 0; col < m; col++) {
                    ap[col] = idx;
                    double expTerm = Math.exp(params[1] * xData[col]);

                    ai[idx] = 0; // derivative w.r.t. a
                    ax[idx] = expTerm;
                    idx++;

                    ai[idx] = 1; // derivative w.r.t. b
                    ax[idx] = params[0] * xData[col] * expTerm;
                    idx++;
                }
                ap[m] = idx;

                printSparseMatrix("Ht (Jacobian transpose)", n, m, ap, ai, ax);

                // 4. Solve
                double[] delta = new double[n];
                cholesky.solve("test", sqrtWeights, residuals, m, n, ap, ai, ax, delta);

                // 5. Update with damping (helps with nonlinear problems)
                double dampingFactor = 1.0;
                for (int i = 0; i < n; i++) {
                    params[i] += dampingFactor * delta[i];
                }

                // 6. Check convergence
                double deltaNorm = 0.0;
                for (double d : delta) {
                    deltaNorm += d * d;
                }
                deltaNorm = Math.sqrt(deltaNorm);

                System.out.printf("Iteration %d: a=%.6f, b=%.6f, ||delta||=%.9f%n",
                        iter, params[0], params[1], deltaNorm);

                if (deltaNorm < tolerance) {
                    System.out.println("Converged!");
                    break;
                }
            }

            // Expected result: a ≈ 5.0, b ≈ -0.5
            assertArrayEquals(new double[]{5.0, -0.5}, params, 0.1,
                    "Parameters should fit exponential decay model");

        } finally {
            cholesky.release("test");
        }
    }
}