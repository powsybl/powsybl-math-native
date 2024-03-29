/**
 * Copyright (c) 2023, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * @file native.cpp
 * @author Geoffroy Jamgotchian <geoffroy.jamgotchian at rte-france.com>
 */

#include "jniwrapper.hpp"
#include "lu.hpp"
#include "solver.hpp"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Class:     com_powsybl_math_AbstractMathNative
 * Method:    nativeInit
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_powsybl_math_AbstractMathNative_nativeInit(JNIEnv * env, jclass) {
    try {
        // lookup caching
        powsybl::jni::ComPowsyblMathMatrixSparseMatrix::init(env);
        powsybl::jni::ComPowsyblMathSolverKinsolContext::init(env);
        powsybl::jni::ComPowsyblMathSolverKinsolResult::init(env);
    } catch (const std::exception& e) {
        powsybl::jni::throwMathException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMathException(env, "Unknown exception");
    }
}

#ifdef __cplusplus
}
#endif
