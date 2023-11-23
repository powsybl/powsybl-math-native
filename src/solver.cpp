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
    _updateFunc = env->GetMethodID(_cls, "updateFunc", "([D[D)V");
    _updateJac = env->GetMethodID(_cls, "updateJac", "([D[I[I[D)V");
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

std::vector<double> copyFromJava(JNIEnv* env, jdoubleArray ja) {
    DoubleArray a(env, ja);
    double* ptr = a.get();
    return std::vector<double>(ptr, ptr + a.length());
}

void copyToJava(JNIEnv* env, jdoubleArray ja, const std::vector<double>& v) {
    DoubleArray a(env, ja);
    std::memcpy(a.get(), v.data(), v.size() * sizeof(double));
}

void ComPowsyblMathSolverNewtonKrylovSolverContext::updateFunc(double* x, double* f, int n) {
    DoubleArray jx(_env, x, n);
    DoubleArray jf(_env, n);
    _env->CallVoidMethod(_obj, _updateFunc, jx.obj(), jf.obj());
    std::memcpy(f, jf.get(), n * sizeof(double));
}

void ComPowsyblMathSolverNewtonKrylovSolverContext::updateJac(double* x, int n, int* ap, int* ai, double* ax, int nnz) {
    DoubleArray jx(_env, x, n);
    IntArray jap(_env, n + 1);
    IntArray jai(_env, nnz);
    DoubleArray jax(_env, nnz);
    _env->CallVoidMethod(_obj, _updateJac, jx.obj(), jap.obj(), jai.obj(), jax.obj());
    std::memcpy(ap, jap.get(), (n + 1) * sizeof(int));
    std::memcpy(ai, jai.get(), nnz * sizeof(int));
    std::memcpy(ax, jax.get(), nnz * sizeof(double));
}

}  // namespace jni

class NewtonKrylovSolverContext {
public:
    NewtonKrylovSolverContext(JNIEnv* env, jobject jobj)
        : _delegate(env, jobj) {
    }

    void logError(int errorCode, const std::string& module, const std::string& function, const std::string& message) {
        _delegate.logError(errorCode, module, function, message);
    }

    void logInfo(const std::string& module, const std::string& function, const std::string& message) {
        _delegate.logInfo(module, function, message);
    }

    void updateFunc(double* x, double* f, int n) {
        _delegate.updateFunc(x, f, n);
    }

    void updateJac(double* x, int n, int* ap, int* ai, double* ax, int nnz) {
        _delegate.updateJac(x, n, ap, ai, ax, nnz);
    }

private:
    powsybl::jni::ComPowsyblMathSolverNewtonKrylovSolverContext _delegate;
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
    int* ap = SUNSparseMatrix_IndexPointers(j);
    int* ai = SUNSparseMatrix_IndexValues(j);
    double* ax = SUNSparseMatrix_Data(j);
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

void solve(std::vector<double>& xd, int nnz, powsybl::NewtonKrylovSolverContext& solverContext,
           int maxIter, bool lineSearch, int level) {
    SUNContext sunCtx;
    int error = SUNContext_Create(nullptr, &sunCtx);
    if (error != 0) {
        throw std::runtime_error("SUNContext_Create error " + std::to_string(error));
    }

    int n = xd.size();
    N_Vector x = N_VMake_Serial(n, xd.data(), sunCtx);

    SUNMatrix j = SUNSparseMatrix(n, n, nnz, CSC_MAT, sunCtx);

    SUNLinearSolver ls = SUNLinSol_KLU(x, j, sunCtx);
    if (!ls) {
        throw std::runtime_error("SUNLinSol_KLU error");
    }

    void* kinMem = KINCreate(sunCtx);
    if (!kinMem) {
        throw std::runtime_error("KINCreate error");
    }

    error = KINInit(kinMem, powsybl::evalFunc, x);
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

    error = KINSetPrintLevel(kinMem, level);
    if (error != KIN_SUCCESS) {
        throw std::runtime_error("KINSetPrintLevel error " + std::to_string(error));
    }

    error = KINSetUserData(kinMem, &solverContext);
    if (error != KIN_SUCCESS) {
        throw std::runtime_error("KINSetUserData error " + std::to_string(error));
    }

    error = KINSetNumMaxIters(kinMem, maxIter);
    if (error != KIN_SUCCESS) {
        throw std::runtime_error("KINSetNumMaxIters error " + std::to_string(error));
    }

    error = KINSetMaxSetupCalls(kinMem, 1);
    if (error != KIN_SUCCESS) {
        throw std::runtime_error("KINSetMaxSetupCalls error " + std::to_string(error));
    }

    N_Vector scale = N_VNew_Serial(n, sunCtx);
    N_VConst(1, scale); // no scale

    error = KINSol(kinMem, x, lineSearch ? KIN_LINESEARCH : KIN_NONE, scale, scale);
    if (error != KIN_SUCCESS) {
        throw std::runtime_error("KINSol error " + std::to_string(error));
    }

    KINFree(&kinMem);

    error = SUNLinSolFree_KLU(ls);
    if (error != 0) {
        throw std::runtime_error("SUNLinSolFree_KLU error " + std::to_string(error));
    }

    SUNMatDestroy(j);

    N_VDestroy_Serial(x);

    error = SUNContext_Free(&sunCtx);
    if (error != 0) {
        throw std::runtime_error("SUNContext_Free error " + std::to_string(error));
    }
}

}

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT void JNICALL Java_com_powsybl_math_solver_NewtonKrylovSolver_solve(JNIEnv * env, jobject, jdoubleArray jx, jint nnz, jobject jSolverContext) {
    try {
        std::vector<double> x = powsybl::jni::copyFromJava(env, jx);
        powsybl::NewtonKrylovSolverContext solverContext(env, jSolverContext);
        int maxIter = 200;
        bool lineSearch = false;
        int level = 2;
        powsybl::solve(x, nnz, solverContext, maxIter, lineSearch, level);
        powsybl::jni::copyToJava(env, jx, x);
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
}

#ifdef __cplusplus
}
#endif
