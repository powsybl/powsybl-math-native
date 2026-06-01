/**
 * Copyright (c) 2023, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.math.matrix;

import com.powsybl.math.AbstractMathNative;

/**
 * @author Geoffroy Jamgotchian {@literal <geoffroy.jamgotchian at rte-france.com>}
 */
public class SparseMatrix extends AbstractMathNative {

    private final int rowCount;
    private final int columnCount;
    private final int[] columnStart;
    private final int[] rowIndices;
    private final double[] values;

    /**
     * Invoked from native code (see the (II[I[I[D)V constructor looked up in
     * lu.cpp) to hand back the result of times/add/transpose. The fields are
     * kept so tests can assert on that result.
     */
    public SparseMatrix(int rowCount, int columnCount, int[] columnStart, int[] rowIndices, double[] values) {
        this.rowCount = rowCount;
        this.columnCount = columnCount;
        this.columnStart = columnStart;
        this.rowIndices = rowIndices;
        this.values = values;
    }

    public int getRowCount() {
        return rowCount;
    }

    public int getColumnCount() {
        return columnCount;
    }

    public int[] getColumnStart() {
        return columnStart;
    }

    public int[] getRowIndices() {
        return rowIndices;
    }

    public double[] getValues() {
        return values;
    }

    public native SparseMatrix times(int m1, int n1, int[] ap1, int[] ai1, double[] ax1, int m2, int n2, int[] ap2, int[] ai2, double[] ax2);

    public native SparseMatrix add(int m1, int n1, int[] ap1, int[] ai1, double[] ax1, int m2, int n2, int[] ap2, int[] ai2, double[] ax2, double alpha, double beta);

    public native SparseMatrix transpose(int m, int n, int[] ap, int[] ai, double[] ax);
}
