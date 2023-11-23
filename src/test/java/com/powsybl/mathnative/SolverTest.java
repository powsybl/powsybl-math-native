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
        int[] ap = new int[] {0, 2, 4};
        int[] ai = new int[] {0, 1, 0, 1};
        double[] ax = new double[4];
        NewtonKrylovSolver solver = new NewtonKrylovSolver();
        solver.solve(x, ap, ai, ax, new NewtonKrylovSolverContext(x, ax));
        assertArrayEquals(new double[] {0.85545, -0.235992}, x, 1e-6);
    }
}
