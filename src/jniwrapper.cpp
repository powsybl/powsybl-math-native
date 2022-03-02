/**
 * Copyright (c) 2017, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * @file jniwrapper.cpp
 * @author Geoffroy Jamgotchian <geoffroy.jamgotchian at rte-france.com>
 */

#include "jniwrapper.hpp"

namespace powsybl {

namespace jni {

void throwMatrixException(JNIEnv* env, const char* msg) {
    jclass clazz = env->FindClass("com/powsybl/math/matrix/MatrixException");
    env->ThrowNew(clazz, msg);
}

}  // namespace jni

}  // namespace powsybl
