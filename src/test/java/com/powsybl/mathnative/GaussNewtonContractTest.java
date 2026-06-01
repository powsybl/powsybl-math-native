/**
 * Copyright (c) 2026, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.mathnative;

import com.powsybl.math.matrix.MatrixException;
import com.powsybl.math.solver.GaussNewtonCholesky;
import com.powsybl.math.solver.NativeBuffers;
import org.junit.jupiter.api.Test;

import java.nio.DoubleBuffer;
import java.nio.IntBuffer;

import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Guards the init() / factorize() dimension contract.
 *
 * The weight array is sized from init's m, but factorize and solve take their
 * own m: without validation an m larger than init's reads past the end of that
 * array. These tests pin the resulting exceptions.
 *
 * @author Gautier Bureau {@literal <gautier.bureau at rte-france.com>}
 */
class GaussNewtonContractTest {

    /** The guard must be what rejected the call - not some incidental failure. */
    private static void assertDimensionMismatch(MatrixException e) {
        assertTrue(e.getMessage().contains("dimension mismatch"),
                "expected the init/factorize dimension guard, got: " + e.getMessage());
    }

    private static final int N = 2;

    /** Ht (2 x m, CSC) for a y = a + b*x fit over x = 0..m-1. */
    private static IntBuffer[] pattern(int m) {
        int[] ap = new int[m + 1];
        int[] ai = new int[2 * m];
        int idx = 0;
        for (int col = 0; col < m; col++) {
            ap[col] = idx;
            ai[idx++] = 0;
            ai[idx++] = 1;
        }
        ap[m] = idx;
        return new IntBuffer[]{NativeBuffers.wrap(ap), NativeBuffers.wrap(ai)};
    }

    private static DoubleBuffer values(int m) {
        double[] ax = new double[2 * m];
        int idx = 0;
        for (int col = 0; col < m; col++) {
            ax[idx++] = 1.0;
            ax[idx++] = col;
        }
        return NativeBuffers.wrap(ax);
    }

    private static double[] unitSqrtWeights(int m) {
        double[] w = new double[m];
        java.util.Arrays.fill(w, 1.0);
        return w;
    }

    /**
     * The out-of-bounds case from review: init declares m = 3, so only 3 squared
     * weights are held, but factorize is handed a 5-row Jacobian.
     */
    @Test
    void factorizeWithLargerMThanInitThrows() {
        int initM = 3;
        int factorizeM = 5;
        GaussNewtonCholesky s = new GaussNewtonCholesky();
        String id = "contract-bigger-m";
        s.init(id, initM, N, unitSqrtWeights(initM));
        try {
            IntBuffer[] p = pattern(factorizeM);
            MatrixException e = assertThrows(MatrixException.class,
                    () -> s.factorize(id, factorizeM, N, p[0], p[1], values(factorizeM)),
                    "factorize with m > init's m must be rejected, not read past the weights");
            assertDimensionMismatch(e);
        } finally {
            s.release(id);
        }
    }

    @Test
    void factorizeWithSmallerMThanInitThrows() {
        int initM = 5;
        int factorizeM = 3;
        GaussNewtonCholesky s = new GaussNewtonCholesky();
        String id = "contract-smaller-m";
        s.init(id, initM, N, unitSqrtWeights(initM));
        try {
            IntBuffer[] p = pattern(factorizeM);
            MatrixException e = assertThrows(MatrixException.class,
                    () -> s.factorize(id, factorizeM, N, p[0], p[1], values(factorizeM)),
                    "factorize with m < init's m silently ignores weights; must be rejected");
            assertDimensionMismatch(e);
        } finally {
            s.release(id);
        }
    }

    @Test
    void factorizeWithMismatchedNThrows() {
        int m = 5;
        GaussNewtonCholesky s = new GaussNewtonCholesky();
        String id = "contract-bad-n";
        s.init(id, m, N, unitSqrtWeights(m));
        try {
            IntBuffer[] p = pattern(m);
            MatrixException e = assertThrows(MatrixException.class,
                    () -> s.factorize(id, m, N + 1, p[0], p[1], values(m)),
                    "factorize with an n differing from init's must be rejected");
            assertDimensionMismatch(e);
        } finally {
            s.release(id);
        }
    }

    @Test
    void solveLMWithLargerMThanInitThrows() {
        int initM = 3;
        int solveM = 5;
        GaussNewtonCholesky s = new GaussNewtonCholesky();
        String id = "contract-lm-m";
        s.init(id, initM, N, unitSqrtWeights(initM));
        try {
            IntBuffer[] p = pattern(solveM);
            DoubleBuffer r = NativeBuffers.allocDouble(solveM);
            DoubleBuffer result = NativeBuffers.allocDouble(N);
            MatrixException e = assertThrows(MatrixException.class,
                    () -> s.solveLM(id, r, solveM, N, p[0], p[1], values(solveM),
                            1e-3, GaussNewtonCholesky.LM_MODE_MARQUARDT, result),
                    "solveLM must enforce the same dimension contract as factorize");
            assertDimensionMismatch(e);
        } finally {
            s.release(id);
        }
    }

    @Test
    void initWithTooFewWeightsThrows() {
        GaussNewtonCholesky s = new GaussNewtonCholesky();
        MatrixException e = assertThrows(MatrixException.class,
                () -> s.init("contract-short-w", 5, N, unitSqrtWeights(3)),
                "init must reject a sqrtWeights array shorter than m");
        assertTrue(e.getMessage().contains("weights are required"),
                "unexpected message: " + e.getMessage());
    }

    @Test
    void initWithNonPositiveDimensionsThrows() {
        GaussNewtonCholesky s = new GaussNewtonCholesky();
        assertThrows(MatrixException.class,
                () -> s.init("contract-zero-m", 0, N, new double[0]),
                "init must reject m = 0");
        assertThrows(MatrixException.class,
                () -> s.init("contract-zero-n", 5, 0, unitSqrtWeights(5)),
                "init must reject n = 0");
    }
}
