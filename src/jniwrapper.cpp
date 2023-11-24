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

void throwKinsolException(JNIEnv* env, const char* msg) {
    throwException(env, msg, "com/powsybl/math/solver/KinsolException");
}

std::vector<double> createDoubleVector(JNIEnv* env, jdoubleArray jda) {
    DoubleArray da(env, jda);
    double* ptr = da.get();
    return std::vector<double>(ptr, ptr + da.length());
}

void updateJavaDoubleArray(JNIEnv* env, jdoubleArray ja, const std::vector<double>& v) {
    DoubleArray a(env, ja);
    std::memcpy(a.get(), v.data(), v.size() * sizeof(double));
}

}  // namespace jni

}  // namespace powsybl
