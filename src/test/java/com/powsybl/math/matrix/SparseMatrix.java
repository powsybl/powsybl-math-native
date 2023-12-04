/**
 * Copyright (c) 2023, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
package com.powsybl.math.matrix;

import com.powsybl.math.AbstractMathNative;

/**
 * @author Geoffroy Jamgotchian {@literal <geoffroy.jamgotchian at rte-france.com>}
 */
public class SparseMatrix extends AbstractMathNative {

    public SparseMatrix(int rowCount, int columnCount, int[] columnStart, int[] rowIndices, double[] values) {
    }

    public native SparseMatrix times(int m1, int n1, int[] ap1, int[] ai1, double[] ax1, int m2, int n2, int[] ap2, int[] ai2, double[] ax2);

    public native SparseMatrix add(int m1, int n1, int[] ap1, int[] ai1, double[] ax1, int m2, int n2, int[] ap2, int[] ai2, double[] ax2, double alpha, double beta);

    public native SparseMatrix transpose(int m, int n, int[] ap, int[] ai, double[] ax);
}
