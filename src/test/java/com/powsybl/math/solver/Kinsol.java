/**
 * Copyright (c) 2023, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
package com.powsybl.math.solver;

import com.powsybl.math.AbstractMathNative;

/**
 * @author Geoffroy Jamgotchian {@literal <geoffroy.jamgotchian at rte-france.com>}
 */
public class Kinsol extends AbstractMathNative {

    public native KinsolResult solve(double[] x, int[] ap, int[] ai, double[] ax, KinsolContext context,
                                     boolean transpose, int maxIters, int msbset, int msbsetsub, double fnormtol,
                                     double scsteptol, boolean lineSearch, int printLevel);
}
