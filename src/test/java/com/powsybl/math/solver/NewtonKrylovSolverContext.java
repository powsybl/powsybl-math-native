/**
 * Copyright (c) 2023, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
package com.powsybl.math.solver;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * @author Geoffroy Jamgotchian {@literal <geoffroy.jamgotchian at rte-france.com>}
 */
public class NewtonKrylovSolverContext {

    private static final Logger LOGGER = LoggerFactory.getLogger(NewtonKrylovSolverContext.class);

    private final double[] x;

    private final double[] ax;

    public NewtonKrylovSolverContext(double[] x, double[] ax) {
        this.x = x;
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

    public void updateFunc(double[] f) {
        // 0 = 0.02 + v2 * 0.1 * sin(ph2)
        // 0 = 0.01 + v2 * 0.1 (-cos(ph2) + v2)
        double v2 = x[0];
        double ph2 = x[1];
        f[0] = 0.02 + v2 * 0.1 * Math.sin(ph2);
        f[1] = 0.01 + v2 * 0.1 * (-Math.cos(ph2) + v2);
    }

    public void updateJac() {
        double v2 = x[0];
        double ph2 = x[1];
        double dp2dv2 = 0.1 * Math.sin(ph2);
        double dp2dph2 = v2 * 0.1 * Math.cos(ph2);
        double dq2dv2 = -0.1 * Math.cos(ph2) + 2 * v2 * 0.1;
        double dq2dph2 = v2 * 0.1 * Math.sin(ph2);
        ax[0] = dp2dv2;
        ax[1] = dp2dph2;
        ax[2] = dq2dv2;
        ax[3] = dq2dph2;
    }
}
