/**
 * Copyright (c) 2023, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
package com.powsybl.mathnative;

import com.powsybl.math.matrix.SparseMatrix;
import com.powsybl.math.solver.NewtonKrylovSolver;
import org.junit.jupiter.api.Test;

/**
 * @author Geoffroy Jamgotchian {@literal <geoffroy.jamgotchian at rte-france.com>}
 */
class SolverTest {

    @Test
    void test() {
        SparseMatrix m = new SparseMatrix(0, 0, new int[] {}, new int[] {}, new double[] {});
        NewtonKrylovSolver solver = new NewtonKrylovSolver();
        solver.test();
    }
}
