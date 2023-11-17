/**
 * Copyright (c) 2023, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
package com.powsybl.math.matrix;

import java.nio.ByteBuffer;

/**
 * @author Geoffroy Jamgotchian {@literal <geoffroy.jamgotchian at rte-france.com>}
 */
public class SparseLUDecomposition {

    public native void init(String id, int[] ap, int[] ai, double[] ax);

    public native void release(String id);

    public native double update(String id, int[] ap, int[] ai, double[] ax, double rgrowthThreshold);

    public native void solve(String id, double[] b, boolean transpose);

    public native void solve2(String id, int m, int n, ByteBuffer b, boolean transpose);
}
