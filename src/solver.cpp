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

    static int eval(N_Vector x, N_Vector values, void* user_data) {
        return 0;
    }
}

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT void JNICALL Java_com_powsybl_math_solver_KinsolSolver_test(JNIEnv * env, jobject) {
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

        // TODO

        KINFree(&kin_mem);

        error = SUNLinSolFree_KLU(ls);
        if (error != 0) {
            throw std::runtime_error("SUNLinSolFree_KLU error " + std::to_string(error));
        }

        SUNMatDestroy(j);

        N_VDestroy_Serial(x0);

        error = SUNContext_Free(&sunctx);
        if (error != 0) {
            throw std::runtime_error("SUNContext_Create error " + std::to_string(error));
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
