/**
 * Copyright (c) 2023, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * @file solver.cpp
 * @author Geoffroy Jamgotchian <geoffroy.jamgotchian at rte-france.com>
 */

#include <iostream>
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

void ComPowsyblMathSolverNewtonKrylovSolverContext::init(JNIEnv* env) {
    jclass localCls = env->FindClass("com/powsybl/math/solver/NewtonKrylovSolverContext");
    _cls = reinterpret_cast<jclass>(env->NewGlobalRef(localCls));
    _logError = env->GetMethodID(_cls, "logError", "(ILjava/lang/String;Ljava/lang/String;Ljava/lang/String;)V");
    _logInfo = env->GetMethodID(_cls, "logInfo", "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;)V");
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

private:
    powsybl::jni::ComPowsyblMathSolverNewtonKrylovSolverContext _delegate;
};

static int evalF(N_Vector x, N_Vector f, void* user_data) {
    std::cout << "eval" << std::endl;
    // 0 = 0.02 + v2 * 0.1 * sin(ph2)
    // 0 = 0.01 + v2 * 0.1 (-cos(ph2) + v2)
    // solution: (0.855373, -0.236001)
    double* xData = N_VGetArrayPointer(x);
    double* fData = N_VGetArrayPointer(f);
    double v2 = xData[0];
    double ph2 = xData[1];
    fData[0] = 0.02 + v2 * 0.1 * std::sin(ph2);
    fData[1] = 0.01 + v2 * 0.1 * (-std::cos(ph2) + v2);
    N_VPrint_Serial(x);
    return 0;
}

static int evalJ(N_Vector x, N_Vector f, SUNMatrix j, void* user_data, N_Vector tmp1, N_Vector tmp2) {
    // TODO
    std::cout << "evalDer" << std::endl;

    double* xData = N_VGetArrayPointer(x);
    double v2 = xData[0];
    double ph2 = xData[1];

    sunindextype* colPtrs = SUNSparseMatrix_IndexPointers(j);
    sunindextype* rowVals = SUNSparseMatrix_IndexValues(j);
    double* data = SUNSparseMatrix_Data(j);

    // p2: 0 = 0.02 + v2 * 0.1 * sin(ph2)
    // q2: 0 = 0.01 + v2 * 0.1 (-cos(ph2) + v2)
    double dp2dv2 = 0.1 * std::sin(ph2);
    double dp2dph2 = v2 * 0.1 * std::cos(ph2);
    double dq2dv2 = - 0.1 * cos(ph2) + 2 * v2 * 0.1;
    double dq2dph2 = v2 * 0.1 * std::sin(ph2);

    std::cout << dp2dv2 << std::endl;
    std::cout << dp2dph2 << std::endl;
    std::cout << dq2dv2 << std::endl;
    std::cout << dq2dph2 << std::endl;

  //  SUNMatZero(j);

    colPtrs[0] = 0;
    colPtrs[1] = 2;
    data[0] = dp2dv2;
    data[1] = dp2dph2;
    data[2] = dq2dv2;
    data[3] = dq2dph2;
    rowVals[0] = 0;
    rowVals[1] = 1;
    rowVals[2] = 0;
    rowVals[3] = 1;

    SUNSparseMatrix_Print(j, stdout);
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

}

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT void JNICALL Java_com_powsybl_math_solver_NewtonKrylovSolver_solve(JNIEnv * env, jobject, jobject jSolverContext) {
    try {
        SUNContext sunCtx;
        int error = SUNContext_Create(nullptr, &sunCtx);
        if (error != 0) {
            throw std::runtime_error("SUNContext_Create error " + std::to_string(error));
        }

        int n = 2;
        int nnz = 4;
        double xData[2] = {1, 0};
        N_Vector x = N_VMake_Serial(n, xData, sunCtx);

        SUNMatrix j = SUNSparseMatrix(n, n, nnz, CSC_MAT, sunCtx);

        SUNLinearSolver ls = SUNLinSol_KLU(x, j, sunCtx);
        if (!ls) {
            throw std::runtime_error("SUNLinSol_KLU error");
        }

        void* kinMem = KINCreate(sunCtx);
        if (!kinMem) {
            throw std::runtime_error("KINCreate error");
        }

        error = KINInit(kinMem, powsybl::evalF, x);
        if (error != KIN_SUCCESS) {
            throw std::runtime_error("KINInit error " + std::to_string(error));
        }

        error = KINSetLinearSolver(kinMem, ls, j);
        if (error != KINLS_SUCCESS) {
            throw std::runtime_error("KINSetLinearSolver error " + std::to_string(error));
        }

        error = KINSetJacFn(kinMem, powsybl::evalJ);
        if (error != KINLS_SUCCESS) {
            throw std::runtime_error("KINSetJacFn error " + std::to_string(error));
        }

        powsybl::NewtonKrylovSolverContext solverContext(env, jSolverContext);

        error = KINSetErrHandlerFn(kinMem, powsybl::errorHandler, &solverContext);
        if (error != KIN_SUCCESS) {
            throw std::runtime_error("KINSetErrHandlerFn error " + std::to_string(error));
        }

        error = KINSetInfoHandlerFn(kinMem, powsybl::infoHandler, &solverContext);
        if (error != KIN_SUCCESS) {
            throw std::runtime_error("KINSetInfoHandlerFn error " + std::to_string(error));
        }

        int level = 2;
        error = KINSetPrintLevel(kinMem, level);
        if (error != KIN_SUCCESS) {
            throw std::runtime_error("KINSetPrintLevel error " + std::to_string(error));
        }

        error = KINSetUserData(kinMem, &solverContext);
        if (error != KIN_SUCCESS) {
            throw std::runtime_error("KINSetUserData error " + std::to_string(error));
        }

        int maxIter = 200;
        error = KINSetNumMaxIters(kinMem, maxIter);
        if (error != KIN_SUCCESS) {
            throw std::runtime_error("KINSetNumMaxIters error " + std::to_string(error));
        }

        error = KINSetMaxSetupCalls(kinMem, 1);
        if (error != KIN_SUCCESS) {
            throw std::runtime_error("KINSetMaxSetupCalls error " + std::to_string(error));
        }

        bool lineSearch = false;
        double scaleData[2] = {1, 1}; // no scale
        N_Vector scale = N_VMake_Serial(n, scaleData, sunCtx);

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
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
}

#ifdef __cplusplus
}
#endif
