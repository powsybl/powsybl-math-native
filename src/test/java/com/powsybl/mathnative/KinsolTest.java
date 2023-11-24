/**
 * Copyright (c) 2023, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
package com.powsybl.mathnative;

import com.powsybl.math.solver.Kinsol;
import com.powsybl.math.solver.KinsolContext;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * @author Geoffroy Jamgotchian {@literal <geoffroy.jamgotchian at rte-france.com>}
 */
class KinsolTest {

    @Test
    void test() {
        double[] x = new double[] {1, 0}; // initial guess
        int[] ap = new int[] {0, 2, 4};
        int[] ai = new int[] {0, 1, 0, 1};
        double[] ax = new double[4];
        Kinsol solver = new Kinsol();
        int maxIterations = 15;
        boolean lineSearch = false;
        int printLevel = 2;
        int status = solver.solve(x, ap, ai, ax, new KinsolContext(x, ax), maxIterations, lineSearch, printLevel);
        assertEquals(0, status);
        assertArrayEquals(new double[] {0.85545, -0.235992}, x, 1e-6);
    }
}
