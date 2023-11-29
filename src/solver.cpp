/**
 * Copyright (c) 2023, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * @file solver.cpp
 * @author Geoffroy Jamgotchian <geoffroy.jamgotchian at rte-france.com>
 */

#include <cstring>
#include <iostream>
#include <vector>
#include <kinsol/kinsol.h>
#include <sundials/sundials_nvector.h>
#include <sunmatrix/sunmatrix_sparse.h>
#include <sunlinsol/sunlinsol_klu.h>
#include <nvector/nvector_serial.h>
#include "jniwrapper.hpp"
#include "solver.hpp"

namespace powsybl {

namespace jni {

jclass ComPowsyblMathSolverKinsolContext::_cls = nullptr;
jmethodID ComPowsyblMathSolverKinsolContext::_logError = nullptr;
jmethodID ComPowsyblMathSolverKinsolContext::_logInfo = nullptr;
jmethodID ComPowsyblMathSolverKinsolContext::_updateFunc = nullptr;
jmethodID ComPowsyblMathSolverKinsolContext::_updateJac = nullptr;

void ComPowsyblMathSolverKinsolContext::init(JNIEnv* env) {
    jclass localCls = env->FindClass("com/powsybl/math/solver/KinsolContext");
    _cls = reinterpret_cast<jclass>(env->NewGlobalRef(localCls));
    _logError = env->GetMethodID(_cls, "logError", "(ILjava/lang/String;Ljava/lang/String;Ljava/lang/String;)V");
    _logInfo = env->GetMethodID(_cls, "logInfo", "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;)V");
    _updateFunc = env->GetMethodID(_cls, "updateFunc", "([D)V");
    _updateJac = env->GetMethodID(_cls, "updateJac", "()V");
}

ComPowsyblMathSolverKinsolContext::ComPowsyblMathSolverKinsolContext(JNIEnv* env, jobject obj)
    : JniWrapper<jobject>(env, obj) {
}

void ComPowsyblMathSolverKinsolContext::logError(int errorCode, const std::string& module, const std::string& function, const std::string& message) {
    _env->CallVoidMethod(_obj,
                         _logError,
                         errorCode,
                         _env->NewStringUTF(module.c_str()),
                         _env->NewStringUTF(function.c_str()),
                         _env->NewStringUTF(message.c_str()));
}

void ComPowsyblMathSolverKinsolContext::logInfo(const std::string& module, const std::string& function, const std::string& message) {
    _env->CallVoidMethod(_obj,
                         _logInfo,
                         _env->NewStringUTF(module.c_str()),
                         _env->NewStringUTF(function.c_str()),
                         _env->NewStringUTF(message.c_str()));
}

void ComPowsyblMathSolverKinsolContext::updateFunc(double* xPtr, double* fPtr, int n, jdoubleArray jx) {
    // update x on Java side in a block to release it on Java side immediately
    {
        DoubleArray x(_env, jx);
        std::memcpy(x.get(), xPtr, n * sizeof(double));
    }

    // call Java side update function callback so that f is updated on Java side
    DoubleArray f(_env, n);
    _env->CallVoidMethod(_obj, _updateFunc, f.obj());

    // update f on C side
    std::memcpy(fPtr, f.get(), n * sizeof(double));
}

void ComPowsyblMathSolverKinsolContext::updateJac(double* xPtr, int n, int* apPtr, int* aiPtr, double* axPtr, int nnz,
                                                  jdoubleArray jx, jintArray jap, jintArray jai, jdoubleArray jax) {
    // update x on Java side in a block to release it on Java side immediately
    {
        DoubleArray x(_env, jx);
        std::memcpy(x.get(), xPtr, n * sizeof(double));
    }

    // call Java side update function callback so that jap, jai and jax are updated on Java side
    _env->CallVoidMethod(_obj, _updateJac);

    // update ap, ai, ax on C side
    IntArray ap(_env, jap);
    IntArray ai(_env, jai);
    DoubleArray ax(_env, jax);
    std::memcpy(apPtr, ap.get(), ap.length() * sizeof(int));
    std::memcpy(aiPtr, ai.get(), ai.length() * sizeof(int));
    std::memcpy(axPtr, ax.get(), ax.length() * sizeof(double));
}

jclass ComPowsyblMathSolverKinsolResult::_cls = nullptr;
jmethodID ComPowsyblMathSolverKinsolResult::_constructor = nullptr;

void ComPowsyblMathSolverKinsolResult::init(JNIEnv* env) {
    jclass localCls = env->FindClass("com/powsybl/math/solver/KinsolResult");
    _cls = reinterpret_cast<jclass>(env->NewGlobalRef(localCls));
    _constructor = env->GetMethodID(_cls, "<init>", "(IJ)V");
}

ComPowsyblMathSolverKinsolResult::ComPowsyblMathSolverKinsolResult(JNIEnv* env, jint status, jlong iterations)
    : JniWrapper<jobject>(env, env->NewObject(_cls, _constructor, status, iterations)) {
}

}  // namespace jni

class KinsolContext {
public:
    KinsolContext(JNIEnv* env, jobject jSolverContext, jdoubleArray jx, jintArray jap, jintArray jai, jdoubleArray jax)
        : _delegate(env, jSolverContext),
          _jx(jx),
          _jap(jap),
          _jai(jai),
          _jax(jax) {
    }

    void logError(int errorCode, const std::string& module, const std::string& function, const std::string& message) {
        _delegate.logError(errorCode, module, function, message);
    }

    void logInfo(const std::string& module, const std::string& function, const std::string& message) {
        _delegate.logInfo(module, function, message);
    }

    void updateFunc(double* xPtr, double* fPtr, int n) {
        _delegate.updateFunc(xPtr, fPtr, n, _jx);
    }

    void updateJac(double* xPtr, int n, int* apPtr, int* aiPtr, double* axPtr, int nnz) {
        _delegate.updateJac(xPtr, n, apPtr, aiPtr, axPtr, nnz, _jx, _jap, _jai, _jax);
    }

private:
    powsybl::jni::ComPowsyblMathSolverKinsolContext _delegate;
    jdoubleArray _jx;
    jintArray _jap;
    jintArray _jai;
    jdoubleArray _jax;
};

static int evalFunc(N_Vector x, N_Vector f, void* user_data) {
    auto* context = (KinsolContext*) user_data;
    double* xPtr = N_VGetArrayPointer(x);
    double* fPtr = N_VGetArrayPointer(f);
    int n = NV_LENGTH_S(x);
    // we do not wrap x pointer to a std::vector to avoid extra copy
    context->updateFunc(xPtr, fPtr, n);
    return 0;
}

static int evalJac(N_Vector x, N_Vector f, SUNMatrix j, void* user_data, N_Vector tmp1, N_Vector tmp2) {
    auto* context = (KinsolContext*) user_data;
    double* xPtr = N_VGetArrayPointer(x);
    int n = NV_LENGTH_S(x);
    double* axPtr = SUNSparseMatrix_Data(j);
    int* apPtr = SUNSparseMatrix_IndexPointers(j);
    int* aiPtr = SUNSparseMatrix_IndexValues(j);
    int nnz = SM_NNZ_S(j);
    // we do not wrap any of theses pointers to a std::vector to avoid extra copy
    context->updateJac(xPtr, n, apPtr, aiPtr, axPtr, nnz);
    return 0;
}

static void errorHandler(int error_code, const char* module, const char* function, char* msg, void* user_data) {
    auto* context = (KinsolContext*) user_data;
    context->logError(error_code, module, function, msg);
}

static void infoHandler(const char* module, const char* function, char* msg, void* user_data) {
    auto* context = (KinsolContext*) user_data;
    context->logInfo(module, function, msg);
}

class SunContext {
public:
    SunContext() {
        int error = SUNContext_Create(nullptr, &sunCtx);
        if (error != 0) {
            throw std::runtime_error("SUNContext_Create error " + std::to_string(error));
        }
    }

    operator SUNContext() {
        return sunCtx;
    }

    ~SunContext() {
        int error = SUNContext_Free(&sunCtx);
        if (error != 0) { // never throw an exception in a destructor
            std::cerr << "SUNContext_Free error " << error << std::endl;
        }
    }
private:
    SUNContext sunCtx; // this is a pointer (see the typedef in sundials code)
};

class SunMatrix {
public:
    SunMatrix(SUNMatrix m)
        : _m(m) {
    }

    operator SUNMatrix() {
        return _m;
    }

    ~SunMatrix() {
        SUNMatDestroy(_m);
    }
private:
    SUNMatrix _m; // this is a pointer (see the typedef in sundials code)
};

class NVectorSerial {
public:
    NVectorSerial(int length, double* ptr, SunContext& sunCtx)
        : _v(N_VMake_Serial(length, ptr, sunCtx)) {
    }

    NVectorSerial(int length, double initialValue, SunContext& sunCtx)
        : _v(N_VNew_Serial(length, sunCtx)) {
        N_VConst(initialValue, _v);
    }

    operator N_Vector() {
        return _v;
    }

    ~NVectorSerial() {
        N_VDestroy_Serial(_v);
    }
private:
    N_Vector _v; // this is a pointer (see the typedef in sundials code)
};

SunMatrix createSparseMatrix(SunContext& sunCtx, JNIEnv* env, jintArray jap, jintArray jai, jdoubleArray jax,
                             bool transpose) {
    jni::IntArray ap(env, jap);
    jni::IntArray ai(env, jai);
    jni::DoubleArray ax(env, jax);
    int n = ap.length() - 1; // ap array has the size of the matrix + 1
    int nnz = ai.length();
    SUNMatrix j = SUNSparseMatrix(n, n, nnz, transpose ? CSR_MAT : CSC_MAT, sunCtx);
    int* apPtr = SUNSparseMatrix_IndexPointers(j);
    int* aiPtr = SUNSparseMatrix_IndexValues(j);
    double* axPtr = SUNSparseMatrix_Data(j);
    // we need to copy sparse matrix arrays to be able to release it on Java side
    std::memcpy(apPtr, ap.get(), ap.length() * sizeof(int));
    std::memcpy(aiPtr, ai.get(), ai.length() * sizeof(int));
    std::memcpy(axPtr, ax.get(), ax.length() * sizeof(double));
    return SunMatrix(j);
}

void solve(SunContext& sunCtx, std::vector<double>& xd, SunMatrix& j, powsybl::KinsolContext& context,
           int maxIters, int msbset, int msbsetsub, double fnormtol, double scsteptol, bool lineSearch, int printLevel,
           int& status, long& iterations) {
    int n = xd.size();
    NVectorSerial x(n, xd.data(), sunCtx);

    SUNLinearSolver ls = SUNLinSol_KLU(x, j, sunCtx);
    if (!ls) {
        throw std::runtime_error("SUNLinSol_KLU error");
    }

    void* kinMem = KINCreate(sunCtx);
    if (!kinMem) {
        throw std::runtime_error("KINCreate error");
    }

    int error = KINInit(kinMem, powsybl::evalFunc, x);
    if (error != KIN_SUCCESS) {
        throw std::runtime_error("KINInit error " + std::to_string(error));
    }

    error = KINSetLinearSolver(kinMem, ls, j);
    if (error != KINLS_SUCCESS) {
        throw std::runtime_error("KINSetLinearSolver error " + std::to_string(error));
    }

    error = KINSetJacFn(kinMem, powsybl::evalJac);
    if (error != KINLS_SUCCESS) {
        throw std::runtime_error("KINSetJacFn error " + std::to_string(error));
    }

    error = KINSetErrHandlerFn(kinMem, powsybl::errorHandler, &context);
    if (error != KIN_SUCCESS) {
        throw std::runtime_error("KINSetErrHandlerFn error " + std::to_string(error));
    }

    error = KINSetInfoHandlerFn(kinMem, powsybl::infoHandler, &context);
    if (error != KIN_SUCCESS) {
        throw std::runtime_error("KINSetInfoHandlerFn error " + std::to_string(error));
    }

    error = KINSetPrintLevel(kinMem, printLevel);
    if (error != KIN_SUCCESS) {
        throw std::runtime_error("KINSetPrintLevel error " + std::to_string(error));
    }

    error = KINSetUserData(kinMem, &context);
    if (error != KIN_SUCCESS) {
        throw std::runtime_error("KINSetUserData error " + std::to_string(error));
    }

    error = KINSetNumMaxIters(kinMem, maxIters);
    if (error != KIN_SUCCESS) {
        throw std::runtime_error("KINSetNumMaxIters error " + std::to_string(error));
    }

    error = KINSetMaxSetupCalls(kinMem, msbset);
    if (error != KIN_SUCCESS) {
        throw std::runtime_error("KINSetMaxSetupCalls error " + std::to_string(error));
    }

    error = KINSetMaxSubSetupCalls(kinMem, msbsetsub);
    if (error != KIN_SUCCESS) {
        throw std::runtime_error("KINSetMaxSubSetupCalls error " + std::to_string(error));
    }

    error = KINSetFuncNormTol(kinMem, fnormtol);
    if (error != KIN_SUCCESS) {
        throw std::runtime_error("KINSetFuncNormTol error " + std::to_string(error));
    }

    error = KINSetScaledStepTol(kinMem, scsteptol);
    if (error != KIN_SUCCESS) {
        throw std::runtime_error("KINSetScaledStepTol error " + std::to_string(error));
    }

    NVectorSerial scale(n, 1, sunCtx); // no scale

    status = KINSol(kinMem, x, lineSearch ? KIN_LINESEARCH : KIN_NONE, scale, scale);

    error = KINGetNumNonlinSolvIters(kinMem, &iterations);
    if (error != KINLS_SUCCESS) {
        throw std::runtime_error("KINGetNumLinIters error " + std::to_string(error));
    }

    KINFree(&kinMem);

    error = SUNLinSolFree_KLU(ls);
    if (error != 0) {
        throw std::runtime_error("SUNLinSolFree_KLU error " + std::to_string(error));
    }
}

}

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT jobject JNICALL Java_com_powsybl_math_solver_Kinsol_solve(JNIEnv* env, jobject,
                                                                    jdoubleArray jx, // state array
                                                                    jintArray jap, jintArray jai, jdoubleArray jax, // jacobian matrix
                                                                    jobject jContext, // Java context
                                                                    jboolean transpose, // to define if we resolve J.X=0  or J.Xt=0
                                                                    jint maxIters, jint msbset, jint msbsetsub, jdouble fnormtol, jdouble scsteptol, jboolean lineSearch, // solver parameters
                                                                    jint printLevel) {
    try {
        powsybl::SunContext sunCtx;

        // we need to copy x array to release it on java side
        std::vector<double> x = powsybl::jni::createDoubleVector(env, jx);

        // j is created by copying the 3 arrays
        powsybl::SunMatrix j = powsybl::createSparseMatrix(sunCtx, env, jap, jai, jax, transpose);

        // run solver
        powsybl::KinsolContext context(env, jContext, jx, jap, jai, jax);
        int status;
        long iterations = 0;
        powsybl::solve(sunCtx, x, j, context, maxIters, msbset, msbsetsub, fnormtol, scsteptol, lineSearch, printLevel, status, iterations);

        // x now contains the solution, we need to update it back on Java side
        powsybl::jni::updateJavaDoubleArray(env, jx, x);

        return powsybl::jni::ComPowsyblMathSolverKinsolResult(env, status, iterations).obj();
    } catch (const std::exception& e) {
        powsybl::jni::throwKinsolException(env, e.what());
    } catch (...) {
        powsybl::jni::throwKinsolException(env, "Unknown exception");
    }
    return nullptr;
}

#ifdef __cplusplus
}
#endif
