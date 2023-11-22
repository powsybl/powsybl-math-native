/**
 * Copyright (c) 2023, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
package com.powsybl.mathnative;

import com.powsybl.math.solver.NewtonKrylovSolver;
import com.powsybl.math.solver.NewtonKrylovSolverContext;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

/**
 * @author Geoffroy Jamgotchian {@literal <geoffroy.jamgotchian at rte-france.com>}
 */
class SolverTest {

    @Test
    void test() {
        double[] x = new double[] {1, 0}; // initial guess
        int n = 2;
        int nnz = 4;
        int[] ap = new int[n + 1];
        int[] ai = new int[nnz];
        double[] ax = new double[nnz];
        NewtonKrylovSolver solver = new NewtonKrylovSolver();
        solver.solve(x, nnz, new NewtonKrylovSolverContext(ap, ai, ax));
        assertArrayEquals(new double[] {0.85545, -0.235992}, x, 1e-6);
    }
}
