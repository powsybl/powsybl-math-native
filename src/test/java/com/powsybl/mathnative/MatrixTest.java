/**
 * Copyright (c) 2023, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
package com.powsybl.mathnative;

import com.powsybl.math.matrix.SparseLUDecomposition;
import com.powsybl.math.matrix.SparseMatrix;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

/**
 * @author Geoffroy Jamgotchian {@literal <geoffroy.jamgotchian at rte-france.com>}
 */
class MatrixTest {

    protected static final double EPSILON = Math.pow(10, -15);

    @Test
    void test() {
        SparseMatrix m = new SparseMatrix(0, 0, new int[] {}, new int[] {}, new double[] {});
        SparseLUDecomposition lu = new SparseLUDecomposition();

        String id = "test";
        int[] ap = new int[] {0, 2, 5, 9, 10, 12};
        int[] ai = new int[] {0, 1, 0, 2, 4, 1, 2, 3, 4, 2, 1, 4};
        double[] ax = new double[] {2.0, 3.0, 3.0, -1.0, 4.0, 4.0, -3.0, 1.0, 2.0, 2.0, 6.0, 1.0};
        lu.init(id, ap, ai, ax);
        double[] b = new double[] {8, 45, -3, 3, 19};
        lu.solve(id, b, false);
        assertArrayEquals(new double[] {1, 2, 3, 4, 5}, b, EPSILON);
        lu.release(id);
    }
}
