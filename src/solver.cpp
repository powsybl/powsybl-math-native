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

namespace powsybl {

    static int eval(N_Vector u, N_Vector f, void* user_data) {
        // TODO
        return 0;
    }

    static int eval_der(N_Vector u, N_Vector fu, SUNMatrix j, void* user_data, N_Vector tmp1, N_Vector tmp2) {
        // TODO
        return 0;
    }

    static void error_handler(int error_code, const char* module, const char* function, char* msg, void* user_data) {
        std::cerr << error_code << " " << module << " " << function << " " << msg << std::endl;
    }
}

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT void JNICALL Java_com_powsybl_math_solver_NewtonKrylovSolver_solve(JNIEnv * env, jobject, jobject) {
    try {
        std::cout << "start" << std::endl;

        SUNContext sunctx;
        int error = SUNContext_Create(nullptr, &sunctx);
        if (error != 0) {
            throw std::runtime_error("SUNContext_Create error " + std::to_string(error));
        }
        // to solve
        // x^2 + y^2 = 10
        // xy - 3 = 0
        int n = 2;
        double x0_data[2] = {0, 0};
        N_Vector x0 = N_VMake_Serial(n, x0_data, sunctx);

        SUNMatrix j = SUNSparseMatrix(n, n, 4, CSC_MAT, sunctx);

        SUNLinearSolver ls = SUNLinSol_KLU(x0, j, sunctx);
        if (!ls) {
            throw std::runtime_error("SUNLinSol_KLU error");
        }

        void* kin_mem = KINCreate(sunctx);
        if (!kin_mem) {
            throw std::runtime_error("KINCreate error");
        }

        error = KINInit(kin_mem, powsybl::eval, x0);
        if (error != KIN_SUCCESS) {
            throw std::runtime_error("KINInit error " + std::to_string(error));
        }

        error = KINSetLinearSolver(kin_mem, ls, j);
        if (error != KINLS_SUCCESS) {
            throw std::runtime_error("KINSetLinearSolver error " + std::to_string(error));
        }

        error = KINSetJacFn(kin_mem, powsybl::eval_der);
        if (error != KINLS_SUCCESS) {
            throw std::runtime_error("KINSetJacFn error " + std::to_string(error));
        }

        error = KINSetErrHandlerFn(kin_mem, powsybl::error_handler, nullptr);
        if (error != KIN_SUCCESS) {
            throw std::runtime_error("KINSetErrHandlerFn error " + std::to_string(error));
        }

        // TODO set max iter, etc

        bool line_search = false;
        N_Vector scale;
        N_VConst(RCONST(1.0), scale); // no scale
        error = KINSol(kin_mem, x0, line_search ? KIN_LINESEARCH : KIN_NONE, scale, scale);
        if (error != KIN_SUCCESS) {
            throw std::runtime_error("KINSol error " + std::to_string(error));
        }

        KINFree(&kin_mem);

        error = SUNLinSolFree_KLU(ls);
        if (error != 0) {
            throw std::runtime_error("SUNLinSolFree_KLU error " + std::to_string(error));
        }

        SUNMatDestroy(j);

        N_VDestroy_Serial(x0);

        error = SUNContext_Free(&sunctx);
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
