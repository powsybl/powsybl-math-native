/**
* Copyright (c) 2026, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * @file gaussnewton.cpp
 * @author Gautier Bureau <gautier.bureau at rte-france.com>
 */

#include <string>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <cstring>
#include <cs.h>
#include <cholmod.h>
#include "jniwrapper.hpp"

class GaussNewtonCHOLMODContext {
public:
    GaussNewtonCHOLMODContext() {
        cholmod_start(&common);
        symbolic = nullptr;
        W_diag = nullptr;
        C = nullptr;
        rhs = nullptr;
        delta_x = nullptr;
    }

    GaussNewtonCHOLMODContext(const GaussNewtonCHOLMODContext&) = delete;

    ~GaussNewtonCHOLMODContext() {
        if (symbolic) {
            cholmod_free_factor(&symbolic, &common);
        }
        if (W_diag) {
            cholmod_free_dense(&W_diag, &common);
        }
        if (C) {
            cholmod_free_sparse(&C, &common);
        }
        if (rhs) {
            cholmod_free_dense(&rhs, &common);
        }
        if (delta_x) {
            cholmod_free_dense(&delta_x, &common);
        }
        cholmod_finish(&common);
    }

    GaussNewtonCHOLMODContext& operator=(const GaussNewtonCHOLMODContext&) = delete;

    std::string error() const;

public:
    cholmod_common common;
    cholmod_factor* symbolic;
    cholmod_dense* W_diag;
    cholmod_sparse* C;
    cholmod_dense* rhs;
    cholmod_dense* delta_x;
};

std::string GaussNewtonCHOLMODContext::error() const {
    switch (common.status) {
        case CHOLMOD_OK: return "CHOLMOD_OK";
        case CHOLMOD_NOT_INSTALLED: return "CHOLMOD_NOT_INSTALLED";
        case CHOLMOD_OUT_OF_MEMORY: return "CHOLMOD_OUT_OF_MEMORY";
        case CHOLMOD_TOO_LARGE: return "CHOLMOD_TOO_LARGE";
        case CHOLMOD_INVALID: return "CHOLMOD_INVALID";
        case CHOLMOD_GPU_PROBLEM: return "CHOLMOD_GPU_PROBLEM";
        case CHOLMOD_NOT_POSDEF: return "CHOLMOD_NOT_POSDEF";
        case CHOLMOD_DSMALL: return "CHOLMOD_DSMALL";
        default: return "Unknown CHOLMOD status: " + std::to_string(common.status);
    }
}

class GaussNewtonCHOLMODContextManager {
public:
    GaussNewtonCHOLMODContextManager() = default;

    GaussNewtonCHOLMODContextManager(const GaussNewtonCHOLMODContextManager&) = delete;

    ~GaussNewtonCHOLMODContextManager() = default;

    GaussNewtonCHOLMODContextManager& operator=(const GaussNewtonCHOLMODContextManager&) = delete;

    GaussNewtonCHOLMODContext& createContext(const std::string& id);

    GaussNewtonCHOLMODContext& findContext(const std::string& id);

    void removeContext(const std::string& id);

private:
    std::map<std::string, std::unique_ptr<GaussNewtonCHOLMODContext>> _contexts;
    std::mutex _mutex;
};

GaussNewtonCHOLMODContext& GaussNewtonCHOLMODContextManager::createContext(const std::string& id) {
    std::lock_guard<std::mutex> lk(_mutex);
    if (_contexts.find(id) != _contexts.end()) {
        throw std::runtime_error("CHOLMOD Context " + id + " already exists");
    }
    std::unique_ptr<GaussNewtonCHOLMODContext> context(new GaussNewtonCHOLMODContext());
    auto it = _contexts.insert(std::make_pair(id, std::move(context)));
    return *it.first->second;
}

GaussNewtonCHOLMODContext& GaussNewtonCHOLMODContextManager::findContext(const std::string& id) {
    std::lock_guard<std::mutex> lk(_mutex);
    auto it = _contexts.find(id);
    if (it == _contexts.end()) {
        throw std::runtime_error("CHOLMOD Context " + id + " not found");
    }
    return *it->second;
}

void GaussNewtonCHOLMODContextManager::removeContext(const std::string& id) {
    std::lock_guard<std::mutex> lk(_mutex);
    _contexts.erase(id);
}

std::unique_ptr<GaussNewtonCHOLMODContextManager> CHOLMOD_MANAGER(new GaussNewtonCHOLMODContextManager());

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Class:     com_powsybl_math_solver_GaussNewtonCholesky
 * Method:    init
 * Signature: (Ljava/lang/String;[I[I[D)V
 */
JNIEXPORT void JNICALL Java_com_powsybl_math_solver_GaussNewtonCholesky_init(JNIEnv * env, jobject, jstring j_id, jint m, jint n, jdoubleArray j_w_diag) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        powsybl::jni::DoubleArray w_diag(env, j_w_diag);

        GaussNewtonCHOLMODContext& context = CHOLMOD_MANAGER->createContext(id);

        context.rhs = cholmod_allocate_dense(n, 1, n, CHOLMOD_REAL, &context.common);
        context.W_diag = cholmod_allocate_dense(m, 1, m, CHOLMOD_REAL, &context.common);

        std::memcpy(context.W_diag->x, w_diag.get(), m * sizeof(double));
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
}

/*
 * Class:     com_powsybl_math_solver_GaussNewtonCholesky
 * Method:    release
 * Signature: (Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_com_powsybl_math_solver_GaussNewtonCholesky_release(JNIEnv* env, jobject, jstring j_id) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();

        CHOLMOD_MANAGER->removeContext(id);
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
}

/*
 * Class:     com_powsybl_math_solver_GaussNewtonCholesky
 * Method:    solve
 * Signature: (Ljava/lang/String;[D[DII[I[I[D)[D
 */
JNIEXPORT void JNICALL Java_com_powsybl_math_solver_GaussNewtonCholesky_solve(JNIEnv* env, jobject, jstring j_id, jdoubleArray j_w_diag, jdoubleArray j_r,
                                                                                               jint m, jint n, jintArray j_ap, jintArray j_ai, jdoubleArray j_ax, jdoubleArray j_result) {
    cholmod_sparse Ht;
    cholmod_dense r_weighted;
    // W_diag is a cholmod_dense* containing sqrt(weights)

    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        powsybl::jni::DoubleArray w_diag(env, j_w_diag);
        powsybl::jni::DoubleArray r_array(env, j_r);
        powsybl::jni::IntArray ap(env, j_ap);
        powsybl::jni::IntArray ai(env, j_ai);
        powsybl::jni::DoubleArray ax(env, j_ax);
        powsybl::jni::DoubleArray result(env, j_result);

        GaussNewtonCHOLMODContext& context = CHOLMOD_MANAGER->findContext(id);

        Ht.nrow = n;
        Ht.ncol = m;
        Ht.nzmax = ax.length();
        Ht.p = ap.get();
        Ht.i = ai.get();
        Ht.x = ax.get();
        Ht.z = nullptr;  // no imaginary part
        Ht.stype = 0;    // symmetric, upper triangular stored
        Ht.itype = CHOLMOD_INT;
        Ht.xtype = CHOLMOD_REAL;
        Ht.dtype = CHOLMOD_DOUBLE;
        Ht.sorted = 1;
        Ht.packed = 1;

        // cholmod_dense* Ht_dense = cholmod_sparse_to_dense(&Ht, &context.common);
        // if (Ht_dense) {
        //     double* values = static_cast<double*>(Ht_dense->x);
        //     size_t nrow = Ht_dense->nrow;
        //     size_t ncol = Ht_dense->ncol;
        //
        //     std::cout << "Matrix Ht (" << nrow << " x " << ncol << "):" << std::endl;
        //     for (size_t i = 0; i < nrow; i++) {
        //         for (size_t j = 0; j < ncol; j++) {
        //             std::cout << values[i + j * nrow] << " ";  // Column-major order
        //         }
        //         std::cout << std::endl;
        //     }
        //
        //     cholmod_free_dense(&Ht_dense, &context.common);
        // }

        // 3. Create r_weighted as a wrapper around j_r data (no allocation)
        r_weighted.nrow = m;
        r_weighted.ncol = 1;
        r_weighted.nzmax = m;
        r_weighted.d = m;
        r_weighted.x = r_array.get();  // Point directly to Java array data
        r_weighted.z = nullptr;
        r_weighted.xtype = CHOLMOD_REAL;
        r_weighted.dtype = CHOLMOD_DOUBLE;

        // 4. Scale r by W in place: r_weighted = W .* r (element-wise multiplication)
        double* r_w_ptr = static_cast<double*>(r_weighted.x);
        double* w_ptr = static_cast<double*>(context.W_diag->x);
        for (size_t i = 0; i < m; i++) {
            r_w_ptr[i] *= w_ptr[i];
            // std::cout << "r_weighted[" << i << "] = " << r_w_ptr[i] << std::endl;
        }

        // 5. Scale H by sqrt(W) -> H_weighted = sqrt(W) * H (scale rows)
        if (cholmod_scale(context.W_diag, CHOLMOD_COL, &Ht, &context.common) == 0) {
            throw std::runtime_error("cholmod_scale error: " + context.error());
        }

        // 7. Form the Normal Equations: C = H' * H and store in context
        if (context.C) {
            cholmod_free_sparse(&context.C, &context.common);
        }
        // cholmod_print_sparse(&Ht, "Ht", &context.common);
        // cholmod_print_dense(&r_weighted, "r_weighted", &context.common);
        // cholmod_print_dense(context.rhs, "r_weighted", &context.common);
        context.C = cholmod_aat(&Ht, nullptr, 0, 1, &context.common);
        context.C->stype = 1;
        if (!context.C) {
            throw std::runtime_error("cholmod_aat error: " + context.error());
        }

        // 8. Form the Right Hand Side: rhs = H' * (W * r) = Ht * r_weighted and store in context
        double alpha[2] = {1.0, 0.0};
        double beta[2] = {0.0, 0.0};
        if (cholmod_sdmult(&Ht, 0, alpha, beta, &r_weighted, context.rhs, &context.common) == 0) {
            throw std::runtime_error("cholmod_sdmult error: " + context.error());
        }

        // double* rhsptr = static_cast<double*>(context.rhs->x);
        // for (size_t i = 0; i < n; i++) {
        //     std::cout << "rhs[" << i << "] = " << rhsptr[i] << std::endl;
        // }
        //
        // cholmod_dense* C_dense = cholmod_sparse_to_dense(context.C, &context.common);
        // if (C_dense) {
        //     double* values = static_cast<double*>(C_dense->x);
        //     size_t nrow = C_dense->nrow;
        //     size_t ncol = C_dense->ncol;
        //
        //     std::cout << "Matrix C (" << nrow << " x " << ncol << "):" << std::endl;
        //     for (size_t i = 0; i < nrow; i++) {
        //         for (size_t j = 0; j < ncol; j++) {
        //             std::cout << values[i + j * nrow] << " ";  // Column-major order
        //         }
        //         std::cout << std::endl;
        //     }
        //
        //     cholmod_free_dense(&C_dense, &context.common);
        // }

        // 9. Analyze and Factorize C
        if (!context.symbolic) {
            context.symbolic = cholmod_analyze(context.C, &context.common);
            if (!context.symbolic) {
                throw std::runtime_error("cholmod_analyze error: " + context.error());
            }
        }

        if (cholmod_factorize(context.C, context.symbolic, &context.common) == 0) {
            throw std::runtime_error("cholmod_factorize error: " + context.error());
        }

        // 8. Solve for delta_x: C * delta_x = b
        // Free previous delta_x if it exists
        if (context.delta_x) {
            cholmod_free_dense(&context.delta_x, &context.common);
        }
        context.delta_x = cholmod_solve(CHOLMOD_A, context.symbolic, context.rhs, &context.common);
        if (!context.delta_x) {
            throw std::runtime_error("cholmod_solve error: " + context.error());
        }

        double* delta_xptr = static_cast<double*>(context.delta_x->x);
        for (size_t i = 0; i < n; i++) {
            std::cout << "delta_x[" << i << "] = " << delta_xptr[i] << std::endl;
        }

        // 12. Copy result to Java array
        std::memcpy(result.get(), context.delta_x->x, n * sizeof(double));
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
}

// /*
//  * Class:     com_powsybl_math_solver_GaussNewtonCholesky
//  * Method:    setWeights
//  * Signature: (Ljava/lang/String;[D)V
//  */
// JNIEXPORT void JNICALL Java_com_powsybl_math_solver_GaussNewtonCholesky_setWeights(JNIEnv * env, jobject, jstring j_id, jdoubleArray j_w_diag) {
//     try {
//         std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
//         powsybl::jni::DoubleArray w_diag(env, j_w_diag);
//
//         GaussNewtonCHOLMODContext& context = CHOLMOD_MANAGER->findContext(id);
//
//         int size = w_diag.length();
//
//         // Allocate or reallocate W_diag if needed
//         if (!context.W_diag || context.W_diag->nrow != size) {
//             if (context.W_diag) {
//                 cholmod_free_dense(&context.W_diag, &context.common);
//             }
//             context.W_diag = cholmod_allocate_dense(size, 1, size, CHOLMOD_REAL, &context.common);
//             if (!context.W_diag) {
//                 throw std::runtime_error("Failed to allocate W_diag vector");
//             }
//         }
//
//         // Copy weight values
//         std::memcpy(context.W_diag->x, w_diag.get(), size * sizeof(double));
//     } catch (const std::exception& e) {
//         powsybl::jni::throwMatrixException(env, e.what());
//     } catch (...) {
//         powsybl::jni::throwMatrixException(env, "Unknown exception");
//     }
// }
//
// /*
//  * Class:     com_powsybl_math_matrix_GaussNewtonCholesky
//  * Method:    allocateDeltaX
//  * Signature: (Ljava/lang/String;I)V
//  */
// JNIEXPORT void JNICALL Java_com_powsybl_math_matrix_GaussNewtonCholesky_allocateRhs(JNIEnv * env, jobject, jstring j_id, jint size) {
//     try {
//         std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
//
//         GaussNewtonCHOLMODContext& context = CHOLMOD_MANAGER->findContext(id);
//
//         // Allocate or reallocate delta_x if needed
//         if (!context.rhs || context.rhs->nrow != size) {
//             if (context.rhs) {
//                 cholmod_free_dense(&context.rhs, &context.common);
//             }
//             context.rhs = cholmod_allocate_dense(size, 1, size, CHOLMOD_REAL, &context.common);
//             if (!context.rhs) {
//                 throw std::runtime_error("Failed to allocate b vector");
//             }
//         }
//     } catch (const std::exception& e) {
//         powsybl::jni::throwMatrixException(env, e.what());
//     } catch (...) {
//         powsybl::jni::throwMatrixException(env, "Unknown exception");
//     }
// }

#ifdef __cplusplus
}
#endif