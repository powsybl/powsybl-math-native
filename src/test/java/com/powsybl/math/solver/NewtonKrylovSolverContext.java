/**
 * Copyright (c) 2023, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
package com.powsybl.math.solver;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Arrays;

/**
 * @author Geoffroy Jamgotchian {@literal <geoffroy.jamgotchian at rte-france.com>}
 */
public class NewtonKrylovSolverContext {

    private static final Logger LOGGER = LoggerFactory.getLogger(NewtonKrylovSolverContext.class);

    private final int[] ap;
    private final int[] ai;
    private final double[] ax;

    public NewtonKrylovSolverContext(int[] ap, int[] ai, double[] ax) {
        this.ap = ap;
        this.ai = ai;
        this.ax = ax;
    }

    public void logError(int errorCode, String module, String function, String message) {
        LOGGER.error("KinSol error: code={}, module='{}', function='{}', message='{}'",
                errorCode, module, function, message);
    }

    public void logInfo(String module, String function, String message) {
        LOGGER.info("KinSol info: module='{}', function='{}', message='{}'",
                module, function, message);
    }

    public void updateFunc(double[] x, double[] f) {
        // 0 = 0.02 + v2 * 0.1 * sin(ph2)
        // 0 = 0.01 + v2 * 0.1 (-cos(ph2) + v2)
        // solution: (0.855373, -0.236001)
        double v2 = x[0];
        double ph2 = x[1];
        f[0] = 0.02 + v2 * 0.1 * Math.sin(ph2);
        f[1] = 0.01 + v2 * 0.1 * (-Math.cos(ph2) + v2);
    }

    public void updateJac(double[] x, int[] ap, int[] ai, double[] ax) {
        double v2 = x[0];
        double ph2 = x[1];
        double dp2dv2 = 0.1 * Math.sin(ph2);
        double dp2dph2 = v2 * 0.1 * Math.cos(ph2);
        double dq2dv2 = - 0.1 * Math.cos(ph2) + 2 * v2 * 0.1;
        double dq2dph2 = v2 * 0.1 * Math.sin(ph2);
        ap[0] = 0;
        ap[1] = 2;
        ap[2] = 4;
        ax[0] = dp2dv2;
        ax[1] = dp2dph2;
        ax[2] = dq2dv2;
        ax[3] = dq2dph2;
        ai[0] = 0;
        ai[1] = 1;
        ai[2] = 0;
        ai[3] = 1;
    }
}
