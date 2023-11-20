/**
 * Copyright (c) 2023, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
package com.powsybl.math.solver;

import com.powsybl.math.matrix.SparseMatrix;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * @author Geoffroy Jamgotchian {@literal <geoffroy.jamgotchian at rte-france.com>}
 */
public class NewtonKrylovSolverContext {

    private static final Logger LOGGER = LoggerFactory.getLogger(NewtonKrylovSolverContext.class);

    public void logError(int errorCode, String module, String function, String message) {
        LOGGER.error("KinSol error: code={}, module='{}', function='{}', message='{}'",
                errorCode, module, function, message);
    }

    public void logInfo(String module, String function, String message) {
        LOGGER.info("KinSol info: module='{}', function='{}', message='{}'",
                module, function, message);
    }

    public void updateX(double[] x) {

    }

    public double[] evalF() {
        return null;
    }

    public SparseMatrix evalJ() {
        return null;
    }
}
