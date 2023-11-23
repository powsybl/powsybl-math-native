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

jclass ComPowsyblMathSolverNewtonKrylovSolverContext::_cls = nullptr;
jmethodID ComPowsyblMathSolverNewtonKrylovSolverContext::_logError = nullptr;
jmethodID ComPowsyblMathSolverNewtonKrylovSolverContext::_logInfo = nullptr;
jmethodID ComPowsyblMathSolverNewtonKrylovSolverContext::_updateFunc = nullptr;
jmethodID ComPowsyblMathSolverNewtonKrylovSolverContext::_updateJac = nullptr;

void ComPowsyblMathSolverNewtonKrylovSolverContext::init(JNIEnv* env) {
    jclass localCls = env->FindClass("com/powsybl/math/solver/NewtonKrylovSolverContext");
    _cls = reinterpret_cast<jclass>(env->NewGlobalRef(localCls));
    _logError = env->GetMethodID(_cls, "logError", "(ILjava/lang/String;Ljava/lang/String;Ljava/lang/String;)V");
    _logInfo = env->GetMethodID(_cls, "logInfo", "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;)V");
    _updateFunc = env->GetMethodID(_cls, "updateFunc", "([D)V");
    _updateJac = env->GetMethodID(_cls, "updateJac", "()V");
}

ComPowsyblMathSolverNewtonKrylovSolverContext::ComPowsyblMathSolverNewtonKrylovSolverContext(JNIEnv* env, jobject obj)
    : JniWrapper<jobject>(env, obj) {
}

void ComPowsyblMathSolverNewtonKrylovSolverContext::logError(int errorCode, const std::string& module, const std::string& function, const std::string& message) {
    _env->CallVoidMethod(_obj,
                         _logError,
                         errorCode,
                         _env->NewStringUTF(module.c_str()),
                         _env->NewStringUTF(function.c_str()),
                         _env->NewStringUTF(message.c_str()));
}

void ComPowsyblMathSolverNewtonKrylovSolverContext::logInfo(const std::string& module, const std::string& function, const std::string& message) {
    _env->CallVoidMethod(_obj,
                         _logInfo,
                         _env->NewStringUTF(module.c_str()),
                         _env->NewStringUTF(function.c_str()),
                         _env->NewStringUTF(message.c_str()));
}

void ComPowsyblMathSolverNewtonKrylovSolverContext::updateFunc(double* x, double* f, int n, jdoubleArray jx) {
    // update x on Java side
    {
        DoubleArray xda(_env, jx);
        std::memcpy(xda.get(), x, n * sizeof(double));
    }

    // call Java side update function callback
    DoubleArray fda(_env, n);
    _env->CallVoidMethod(_obj, _updateFunc, fda.obj());

    // update f on C side
    std::memcpy(f, fda.get(), n * sizeof(double));
}

void ComPowsyblMathSolverNewtonKrylovSolverContext::updateJac(double* x, int n, int* ap, int* ai, double* ax, int nnz,
                                                              jdoubleArray jx, jintArray jap, jintArray jai, jdoubleArray jax) {
    // update x on Java side
    {
        DoubleArray xda(_env, jx);
        std::memcpy(xda.get(), x, n * sizeof(double));
    }

    // call Java side update function callback
    _env->CallVoidMethod(_obj, _updateJac);

    // update ax on C side
    IntArray apia(_env, jap);
    IntArray aiia(_env, jai);
    DoubleArray axda(_env, jax);
    std::memcpy(ap, apia.get(), apia.length() * sizeof(int));
    std::memcpy(ai, aiia.get(), aiia.length() * sizeof(int));
    std::memcpy(ax, axda.get(), axda.length() * sizeof(double));
}

}  // namespace jni

class NewtonKrylovSolverContext {
public:
    NewtonKrylovSolverContext(JNIEnv* env, jobject jSolverContext, jdoubleArray jx, jintArray jap, jintArray jai, jdoubleArray jax)
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

    void updateFunc(double* x, double* f, int n) {
        _delegate.updateFunc(x, f, n, _jx);
    }

    void updateJac(double* x, int n, int* ap, int* ai, double* ax, int nnz) {
        _delegate.updateJac(x, n, ap, ai, ax, nnz, _jx, _jap, _jai, _jax);
    }

private:
    powsybl::jni::ComPowsyblMathSolverNewtonKrylovSolverContext _delegate;
    jdoubleArray _jx;
    jintArray _jap;
    jintArray _jai;
    jdoubleArray _jax;
};

static int evalFunc(N_Vector x, N_Vector f, void* user_data) {
    NewtonKrylovSolverContext* solverContext = (NewtonKrylovSolverContext*) user_data;
    double* xd = N_VGetArrayPointer(x);
    double* fd = N_VGetArrayPointer(f);
    int n = NV_LENGTH_S(x);
    solverContext->updateFunc(xd, fd, n);
    return 0;
}

static int evalJac(N_Vector x, N_Vector f, SUNMatrix j, void* user_data, N_Vector tmp1, N_Vector tmp2) {
    NewtonKrylovSolverContext* solverContext = (NewtonKrylovSolverContext*) user_data;
    double* xd = N_VGetArrayPointer(x);
    int n = NV_LENGTH_S(x);
    double* ax = SUNSparseMatrix_Data(j);
    int* ap = SUNSparseMatrix_IndexPointers(j);
    int* ai = SUNSparseMatrix_IndexValues(j);
    int nnz = SM_NNZ_S(j);
    solverContext->updateJac(xd, n, ap, ai, ax, nnz);
    return 0;
}

static void errorHandler(int error_code, const char* module, const char* function, char* msg, void* user_data) {
    NewtonKrylovSolverContext* solverContext = (NewtonKrylovSolverContext*) user_data;
    solverContext->logError(error_code, module, function, msg);
}

static void infoHandler(const char* module, const char* function, char* msg, void* user_data) {
    NewtonKrylovSolverContext* solverContext = (NewtonKrylovSolverContext*) user_data;
    solverContext->logInfo(module, function, msg);
}

SUNMatrix createSparseMatrix(SUNContext& sunCtx, JNIEnv* env, jintArray jap, jintArray jai, jdoubleArray jax) {
    jni::IntArray apia(env, jap);
    jni::IntArray aiia(env, jai);
    jni::DoubleArray axda(env, jax);
    int n = apia.length() - 1;
    int nnz = aiia.length();
    SUNMatrix j = SUNSparseMatrix(n, n, nnz, CSC_MAT, sunCtx);
    int* ap = SUNSparseMatrix_IndexPointers(j);
    int* ai = SUNSparseMatrix_IndexValues(j);
    double* ax = SUNSparseMatrix_Data(j);
    std::memcpy(ap, apia.get(), apia.length() * sizeof(int));
    std::memcpy(ai, aiia.get(), aiia.length() * sizeof(int));
    std::memcpy(ax, axda.get(), axda.length() * sizeof(double));
    return j;
}

int solve(SUNContext& sunCtx, std::vector<double>& xd, SUNMatrix& j, powsybl::NewtonKrylovSolverContext& solverContext,
          int maxIterations, bool lineSearch, int printLevel) {
    int n = xd.size();
    N_Vector x = N_VMake_Serial(n, xd.data(), sunCtx);

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

    error = KINSetErrHandlerFn(kinMem, powsybl::errorHandler, &solverContext);
    if (error != KIN_SUCCESS) {
        throw std::runtime_error("KINSetErrHandlerFn error " + std::to_string(error));
    }

    error = KINSetInfoHandlerFn(kinMem, powsybl::infoHandler, &solverContext);
    if (error != KIN_SUCCESS) {
        throw std::runtime_error("KINSetInfoHandlerFn error " + std::to_string(error));
    }

    error = KINSetPrintLevel(kinMem, printLevel);
    if (error != KIN_SUCCESS) {
        throw std::runtime_error("KINSetPrintLevel error " + std::to_string(error));
    }

    error = KINSetUserData(kinMem, &solverContext);
    if (error != KIN_SUCCESS) {
        throw std::runtime_error("KINSetUserData error " + std::to_string(error));
    }

    error = KINSetNumMaxIters(kinMem, maxIterations);
    if (error != KIN_SUCCESS) {
        throw std::runtime_error("KINSetNumMaxIters error " + std::to_string(error));
    }

    error = KINSetMaxSetupCalls(kinMem, 1);
    if (error != KIN_SUCCESS) {
        throw std::runtime_error("KINSetMaxSetupCalls error " + std::to_string(error));
    }

    N_Vector scale = N_VNew_Serial(n, sunCtx);
    N_VConst(1, scale); // no scale

    int status = KINSol(kinMem, x, lineSearch ? KIN_LINESEARCH : KIN_NONE, scale, scale);

    KINFree(&kinMem);

    error = SUNLinSolFree_KLU(ls);
    if (error != 0) {
        throw std::runtime_error("SUNLinSolFree_KLU error " + std::to_string(error));
    }

    SUNMatDestroy(j);

    N_VDestroy_Serial(x);

    return status;
}

}

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT jint JNICALL Java_com_powsybl_math_solver_NewtonKrylovSolver_solve(JNIEnv* env, jobject, jdoubleArray jx, jintArray jap,
                                                                             jintArray jai, jdoubleArray jax, jobject jSolverContext,
                                                                             jint maxIterations, jboolean lineSearch, jint printLevel) {
    int status = -1;
    try {
        SUNContext sunCtx;
        int error = SUNContext_Create(nullptr, &sunCtx);
        if (error != 0) {
            throw std::runtime_error("SUNContext_Create error " + std::to_string(error));
        }

        std::vector<double> x = powsybl::jni::createDoubleVector(env, jx);
        SUNMatrix j = powsybl::createSparseMatrix(sunCtx, env, jap, jai, jax);

        // run solver
        powsybl::NewtonKrylovSolverContext solverContext(env, jSolverContext, jx, jap, jai, jax);
        status = powsybl::solve(sunCtx, x, j, solverContext, maxIterations, lineSearch, printLevel);

        powsybl::jni::updateJavaDoubleArray(env, jx, x);

        error = SUNContext_Free(&sunCtx);
        if (error != 0) {
            throw std::runtime_error("SUNContext_Free error " + std::to_string(error));
        }
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
    return status;
}

#ifdef __cplusplus
}
#endif
