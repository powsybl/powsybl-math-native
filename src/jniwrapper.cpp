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

void throwException(JNIEnv* env, const char* msg, const std::string& className) {
    jclass clazz = env->FindClass(className.c_str());
    env->ThrowNew(clazz, msg);
}

void throwMathException(JNIEnv* env, const char* msg) {
    throwException(env, msg, "com/powsybl/math/MathException");
}

void throwMatrixException(JNIEnv* env, const char* msg) {
    throwException(env, msg, "com/powsybl/math/matrix/MatrixException");
}

void throwSolverException(JNIEnv* env, const char* msg) {
    throwException(env, msg, "com/powsybl/math/solver/SolverException");
}

}  // namespace jni

}  // namespace powsybl
