/**
 * Copyright (c) 2023, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
package com.powsybl.mathnative;

import com.powsybl.math.solver.Kinsol;
import com.powsybl.math.solver.KinsolContext;
import com.powsybl.math.solver.KinsolResult;
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
        int maxIters = 200;
        int msbset = 0; // default value
        int msbsetsub = 0; // default value
        double fnormtol = 0; // default value
        double scsteptol = 0; // default value
        boolean lineSearch = false;
        int printLevel = 2;
        KinsolResult result = solver.solve(x, ap, ai, ax, new KinsolContext(x, ax), false, maxIters, msbset, msbsetsub, fnormtol, scsteptol, lineSearch, printLevel);
        assertEquals(0, result.getStatus());
        assertEquals(9, result.getIterations());
        assertArrayEquals(new double[] {0.855419, -0.235959}, x, 1e-6);
    }
}
