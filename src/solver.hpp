/**
 * Copyright (c) 2023, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * @file solver.hpp
 * @author Geoffroy Jamgotchian <geoffroy.jamgotchian at rte-france.com>
 */

namespace powsybl {

namespace jni {

class ComPowsyblMathSolverNewtonKrylovSolverContext : public JniWrapper<jobject> {
public:
    ComPowsyblMathSolverNewtonKrylovSolverContext(JNIEnv* env, jobject ob);

    static void init(JNIEnv* env);

    void logError(int errorCode, const std::string& module, const std::string& function, const std::string& message);

    void logInfo(const std::string& module, const std::string& function, const std::string& message);

    void updateFunc(double* x, double* f, int n);

    void updateJac(double* x, int n, int* ap, int* ai, double* ax, int nnz);

private:
    static jclass _cls;
    static jmethodID _logError;
    static jmethodID _logInfo;
    static jmethodID _updateFunc;
    static jmethodID _updateJac;
};

}  // namespace jni

}  // namespace powsybl
