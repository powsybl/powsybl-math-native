/**
 * Copyright (c) 2026, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.mathnative;

import com.powsybl.math.matrix.SparseMatrix;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Covers the CXSparse-backed times / add / transpose entry points, which had no
 * test, and pins the contract they rely on: their input arrays are read-only on
 * the native side, so the caller's arrays must come back untouched.
 *
 * @author Gautier Bureau {@literal <gautier.bureau at rte-france.com>}
 */
class SparseMatrixOpsTest {

    private static final double EPSILON = Math.pow(10, -15);

    // A = [[1, 3],
    //      [2, 4]]  in CSC.
    private static final int[] AP = {0, 2, 4};
    private static final int[] AI = {0, 1, 0, 1};
    private static final double[] AX = {1.0, 2.0, 3.0, 4.0};

    private static SparseMatrix newMatrix() {
        return new SparseMatrix(0, 0, new int[] {}, new int[] {}, new double[] {});
    }

    @Test
    void transposeGivesTransposeAndLeavesInputUntouched() {
        int[] ap = AP.clone();
        int[] ai = AI.clone();
        double[] ax = AX.clone();

        SparseMatrix at = newMatrix().transpose(2, 2, ap, ai, ax);

        // A' = [[1, 2],
        //       [3, 4]]
        assertEquals(2, at.getRowCount());
        assertEquals(2, at.getColumnCount());
        assertArrayEquals(new int[] {0, 2, 4}, at.getColumnStart());
        assertArrayEquals(new int[] {0, 1, 0, 1}, at.getRowIndices());
        assertArrayEquals(new double[] {1.0, 3.0, 2.0, 4.0}, at.getValues(), EPSILON);

        assertInputsUntouched(ap, ai, ax);
    }

    @Test
    void addGivesSumAndLeavesInputsUntouched() {
        int[] ap1 = AP.clone();
        int[] ai1 = AI.clone();
        double[] ax1 = AX.clone();
        int[] ap2 = AP.clone();
        int[] ai2 = AI.clone();
        double[] ax2 = AX.clone();

        SparseMatrix sum = newMatrix().add(2, 2, ap1, ai1, ax1, 2, 2, ap2, ai2, ax2, 1.0, 1.0);

        // 1*A + 1*A = 2A
        assertArrayEquals(new double[] {2.0, 4.0, 6.0, 8.0}, sum.getValues(), EPSILON);

        assertInputsUntouched(ap1, ai1, ax1);
        assertInputsUntouched(ap2, ai2, ax2);
    }

    @Test
    void timesGivesProductAndLeavesInputsUntouched() {
        int[] ap1 = AP.clone();
        int[] ai1 = AI.clone();
        double[] ax1 = AX.clone();
        int[] ap2 = AP.clone();
        int[] ai2 = AI.clone();
        double[] ax2 = AX.clone();

        SparseMatrix product = newMatrix().times(2, 2, ap1, ai1, ax1, 2, 2, ap2, ai2, ax2);

        // A * A = [[7, 15],
        //          [10, 22]]  -> CSC values are column-major
        assertArrayEquals(new double[] {7.0, 10.0, 15.0, 22.0}, product.getValues(), EPSILON);

        assertInputsUntouched(ap1, ai1, ax1);
        assertInputsUntouched(ap2, ai2, ax2);
    }

    private static void assertInputsUntouched(int[] ap, int[] ai, double[] ax) {
        assertArrayEquals(AP, ap, "column pointers must not be modified by the native side");
        assertArrayEquals(AI, ai, "row indices must not be modified by the native side");
        assertArrayEquals(AX, ax, EPSILON, "values must not be modified by the native side");
    }
}
