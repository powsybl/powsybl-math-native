/**
 * Copyright (c) 2023, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * @file solver.hpp
 * @author Geoffroy Jamgotchian <geoffroy.jamgotchian at rte-france.com>
 */

#ifndef SOLVER_HPP
#define SOLVER_HPP

namespace powsybl {

namespace jni {

class ComPowsyblMathSolverKinsolContext : public JniWrapper<jobject> {
public:
    ComPowsyblMathSolverKinsolContext(JNIEnv* env, jobject ob);

    static void init(JNIEnv* env);

    void logError(int errorCode, const std::string& module, const std::string& function, const std::string& message);

    void logInfo(const std::string& module, const std::string& function, const std::string& message);

    void updateFunc(double* x, double* f, int n, jdoubleArray jx);

    void updateJac(double* x, int n, int* ap, int* ai, double* ax, int nnz, jdoubleArray jx, jintArray jap, jintArray jai, jdoubleArray jax);

private:
    static jclass _cls;
    static jmethodID _logError;
    static jmethodID _logInfo;
    static jmethodID _updateFunc;
    static jmethodID _updateJac;
};

class ComPowsyblMathSolverKinsolResult : public JniWrapper<jobject> {
public:
    ComPowsyblMathSolverKinsolResult(JNIEnv* env, jint status, jlong iterations);

    static void init(JNIEnv* env);

private:
    static jclass _cls;
    static jmethodID _constructor;
};

}  // namespace jni

}  // namespace powsybl

#endif // SOLVER_HPP
