package com.powsybl.mathnative;

import com.powsybl.math.solver.GaussNewtonKLU;
import com.powsybl.math.solver.NativeBuffers;
import org.junit.jupiter.api.Test;

import java.nio.DoubleBuffer;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

/**
 * Test for Gauss-Newton least squares minimization using KLU.
 *
 * Problem: Fit a simple exponential model y = a * exp(b * x) to noisy data
 * We'll linearize it and use Gauss-Newton iterations.
 *
 * @author Gautier Bureau {@literal <gautier.bureau at rte-france.com>}
 */
class GaussNewtonKLUTest {

    /**
     * Simple test: fit a line y = a + b*x to data points
     * This is a linear problem so it should converge in 1 iteration.
     */
    @Test
    void testLinearLeastSquares() {
        double[] xData = {0.0, 1.0, 2.0, 3.0, 4.0};
        double[] yData = {1.1, 2.9, 5.2, 6.8, 9.1};
        int m = xData.length;
        int n = 2;

        double[] params = {1.0, 1.0};

        double[] sqrtWeights = new double[m];
        for (int i = 0; i < m; i++) {
            sqrtWeights[i] = 1.0;
        }

        GaussNewtonKLU klu = new GaussNewtonKLU();
        klu.init("test", m, n, sqrtWeights);

        try {
            int maxIter = 10;
            double tolerance = 1e-6;

            for (int iter = 0; iter < maxIter; iter++) {
                double[] residuals = new double[m];
                for (int i = 0; i < m; i++) {
                    double predicted = params[0] + params[1] * xData[i];
                    residuals[i] = yData[i] - predicted;
                }

                int[] ap = new int[m + 1];
                int[] ai = new int[m * n];
                double[] ax = new double[m * n];

                int idx = 0;
                for (int col = 0; col < m; col++) {
                    ap[col] = idx;
                    ai[idx] = 0;
                    ax[idx] = 1.0;
                    idx++;
                    ai[idx] = 1;
                    ax[idx] = xData[col];
                    idx++;
                }
                ap[m] = idx;

                double[] delta = new double[n];
                DoubleBuffer deltaBuf = NativeBuffers.allocDouble(n);
                klu.solve("test", NativeBuffers.wrap(residuals), m, n,
                        NativeBuffers.wrap(ap), NativeBuffers.wrap(ai),
                        NativeBuffers.wrap(ax), deltaBuf);
                NativeBuffers.copyTo(deltaBuf, delta);

                for (int i = 0; i < n; i++) {
                    params[i] += delta[i];
                }

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

            assertArrayEquals(new double[]{1.02, 2.0}, params, 0.1,
                    "Parameters should fit the line y = 1 + 2*x");

        } finally {
            klu.release("test");
        }
    }

    /**
     * More complex test: fit an exponential decay model
     * y = a * exp(-b * x) to data
     */
    @Test
    void testNonlinearLeastSquares() {
        double[] xData = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
        double[] yData = {5.0, 3.03, 1.84, 1.12, 0.68, 0.41};
        int m = xData.length;
        int n = 2;

        double[] params = {1.0, -0.1};

        double[] sqrtWeights = new double[m];
        for (int i = 0; i < m; i++) {
            sqrtWeights[i] = 1.0;
        }

        GaussNewtonKLU klu = new GaussNewtonKLU();
        klu.init("test", m, n, sqrtWeights);

        try {
            int maxIter = 20;
            double tolerance = 1e-6;

            for (int iter = 0; iter < maxIter; iter++) {
                double[] residuals = new double[m];
                for (int i = 0; i < m; i++) {
                    double predicted = params[0] * Math.exp(params[1] * xData[i]);
                    residuals[i] = yData[i] - predicted;
                }

                int[] ap = new int[m + 1];
                int[] ai = new int[m * n];
                double[] ax = new double[m * n];

                int idx = 0;
                for (int col = 0; col < m; col++) {
                    ap[col] = idx;
                    double expTerm = Math.exp(params[1] * xData[col]);

                    ai[idx] = 0;
                    ax[idx] = expTerm;
                    idx++;

                    ai[idx] = 1;
                    ax[idx] = params[0] * xData[col] * expTerm;
                    idx++;
                }
                ap[m] = idx;

                double[] delta = new double[n];
                DoubleBuffer deltaBuf = NativeBuffers.allocDouble(n);
                klu.solve("test", NativeBuffers.wrap(residuals), m, n,
                        NativeBuffers.wrap(ap), NativeBuffers.wrap(ai),
                        NativeBuffers.wrap(ax), deltaBuf);
                NativeBuffers.copyTo(deltaBuf, delta);

                double dampingFactor = 1.0;
                for (int i = 0; i < n; i++) {
                    params[i] += dampingFactor * delta[i];
                }

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

            assertArrayEquals(new double[]{5.0, -0.5}, params, 0.1,
                    "Parameters should fit exponential decay model");

        } finally {
            klu.release("test");
        }
    }
}
