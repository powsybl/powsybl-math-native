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

void ComPowsyblMathSolverNewtonKrylovSolverContext::init(JNIEnv* env) {
    jclass localCls = env->FindClass("com/powsybl/math/solver/NewtonKrylovSolverContext");
    _cls = reinterpret_cast<jclass>(env->NewGlobalRef(localCls));
    _logError = env->GetMethodID(_cls, "logError", "(ILjava/lang/String;Ljava/lang/String;Ljava/lang/String;)V");
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

}  // namespace jni

class NewtonKrylovSolverContext {
public:
    NewtonKrylovSolverContext(JNIEnv* env, jobject jobj)
        : _delegate(env, jobj) {
    }

    void logError(int errorCode, const std::string& module, const std::string& function, const std::string& message) {
        _delegate.logError(errorCode, module, function, message);
    }

private:
    powsybl::jni::ComPowsyblMathSolverNewtonKrylovSolverContext _delegate;
};

static int eval(N_Vector x, N_Vector f, void* user_data) {
    std::cout << "eval" << std::endl;
    // TODO
    // a = x^2 + y^2 - 10
    // b = xy - 3 = 0
    double* xData = N_VGetArrayPointer(x);
    double* fData = N_VGetArrayPointer(f);
    double xx = xData[0];
    double y = xData[1];
    fData[0] = xx * xx + y * y - 10;
    fData[1] = xx * y - 3;
    return 0;
}

static int evalDer(N_Vector x, N_Vector f, SUNMatrix j, void* user_data, N_Vector tmp1, N_Vector tmp2) {
    // TODO
    std::cout << "evalDer" << std::endl;
    // a: x^2 + y^2 - 10
    // b: xy - 3 = 0
    // da/dx = 2x
    // da/dy = 2y
    // db/dx = y
    // db/dy = x
    //
    // 2x  2y
    // y   x
    //
    double* xData = N_VGetArrayPointer(x);
    sunindextype* colPtrs = SUNSparseMatrix_IndexPointers(j);
    sunindextype* rowVals = SUNSparseMatrix_IndexValues(j);
    double* data = SUNSparseMatrix_Data(j);
    SUNMatZero(j);

    colPtrs[0] = 0;
    colPtrs[1] = 2;
    double xx = xData[0];
    double y = xData[1];
    data[0] = 2 * xx;
    data[1] = y;
    data[2] = 2 * y;
    data[3] = xx;
    rowVals[0] = 0;
    rowVals[1] = 1;
    rowVals[2] = 0;
    rowVals[3] = 1;

    std::cout << "fin evalDer" << std::endl;
    return 0;
}

static void errorHandler(int error_code, const char* module, const char* function, char* msg, void* user_data) {
    NewtonKrylovSolverContext* solverContext = (NewtonKrylovSolverContext*) user_data;
    solverContext->logError(error_code, module, function, msg);
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
        // to solve
        // x^2 + y^2 = 10
        // xy - 3 = 0
        int n = 2;
        int nnz = 4;
        double xData[2] = {0, 0};
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

        error = KINInit(kinMem, powsybl::eval, x);
        if (error != KIN_SUCCESS) {
            throw std::runtime_error("KINInit error " + std::to_string(error));
        }

        error = KINSetLinearSolver(kinMem, ls, j);
        if (error != KINLS_SUCCESS) {
            throw std::runtime_error("KINSetLinearSolver error " + std::to_string(error));
        }

        error = KINSetJacFn(kinMem, powsybl::evalDer);
        if (error != KINLS_SUCCESS) {
            throw std::runtime_error("KINSetJacFn error " + std::to_string(error));
        }

        powsybl::NewtonKrylovSolverContext solverContext(env, jSolverContext);

        error = KINSetErrHandlerFn(kinMem, powsybl::errorHandler, &solverContext);
        if (error != KIN_SUCCESS) {
            throw std::runtime_error("KINSetErrHandlerFn error " + std::to_string(error));
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
