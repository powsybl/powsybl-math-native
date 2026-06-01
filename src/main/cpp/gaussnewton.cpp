/**
 * Copyright (c) 2026, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 *
 * @file gaussnewton.cpp
 * @author Gautier Bureau <gautier.bureau at rte-france.com>
 */

#include <string>
#include <memory>
#include <cstring>
#include <stdexcept>
#include <cholmod.h>
#include "jniwrapper.hpp"
#include "context_manager.hpp"
#include "gaussnewton_solver.hpp"

using namespace powsybl::gaussnewton;

class GaussNewtonCHOLMODContext : public ContextBase {
public:
    GaussNewtonCHOLMODContext()
        : symbolic(nullptr), X(nullptr), Y(nullptr), E(nullptr) {
        cholmod_start(&common);
        // Force the (BLAS-free) simplicial factorization. The Supernodal module
        // is the only part of CHOLMOD that needs BLAS/LAPACK, and it is disabled
        // at build time (NSUPERNODAL) to keep the native library portable -
        // notably on Windows where BLAS is painful to build. This runtime
        // setting makes the intent explicit and stays correct even if a
        // supernodal-capable CHOLMOD is ever linked.
        common.supernodal = CHOLMOD_SIMPLICIAL;
    }

    GaussNewtonCHOLMODContext(const GaussNewtonCHOLMODContext&) = delete;

    ~GaussNewtonCHOLMODContext() {
        if (symbolic) {
            cholmod_free_factor(&symbolic, &common);
        }
        if (X) {
            cholmod_free_dense(&X, &common);
        }
        if (Y) {
            cholmod_free_dense(&Y, &common);
        }
        if (E) {
            cholmod_free_dense(&E, &common);
        }
        cholmod_finish(&common);
    }

    GaussNewtonCHOLMODContext& operator=(const GaussNewtonCHOLMODContext&) = delete;

    std::string error() const;

    // Backend hook: analyze (lazy, once) + factorize the current C in normalEq.
    // Sets lastRank to n on success, or to symbolic->minor on CHOLMOD_NOT_POSDEF.
    int analyzeFactor(int n);

    // Backend hook: solve C * x = rhs into result (length n).
    void solveLinear(int n, const double* rhs, double* result);

public:
    cholmod_common common;
    cholmod_factor* symbolic;
    // Workspaces reused across cholmod_solve2 calls (allocated lazily on first solve).
    cholmod_dense* X;
    cholmod_dense* Y;
    cholmod_dense* E;
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

int GaussNewtonCHOLMODContext::analyzeFactor(int n) {
    cholmod_sparse C{};
    C.nrow = n;
    C.ncol = n;
    C.nzmax = normalEq.nnzC();
    C.p = normalEq.cp();
    C.i = normalEq.ci();
    C.x = normalEq.cx();
    C.z = nullptr;
    C.stype = 1;
    C.itype = CHOLMOD_INT;
    C.xtype = CHOLMOD_REAL;
    C.dtype = CHOLMOD_DOUBLE;
    C.sorted = 1;
    C.packed = 1;

    if (!symbolic) {
        symbolic = cholmod_analyze(&C, &common);
        if (!symbolic) {
            throw std::runtime_error("cholmod_analyze error: " + error());
        }
    }

    cholmod_factorize(&C, symbolic, &common);

    int rank;
    if (common.status == CHOLMOD_NOT_POSDEF) {
        rank = static_cast<int>(symbolic->minor);
    } else if (common.status != CHOLMOD_OK) {
        throw std::runtime_error("cholmod_factorize error: " + error());
    } else {
        rank = n;
    }
    lastRank = rank;
    return rank;
}

void GaussNewtonCHOLMODContext::solveLinear(int n, const double* rhs, double* result) {
    cholmod_dense b{};
    b.nrow = n;
    b.ncol = 1;
    b.nzmax = n;
    b.d = n;
    b.x = const_cast<double*>(rhs);
    b.z = nullptr;
    b.xtype = CHOLMOD_REAL;
    b.dtype = CHOLMOD_DOUBLE;

    if (cholmod_solve2(CHOLMOD_A, symbolic, &b, nullptr,
                       &X, nullptr, &Y, &E, &common) == 0) {
        throw std::runtime_error("cholmod_solve2 error: " + error());
    }
    std::memcpy(result, X->x, n * sizeof(double));
}

using GaussNewtonCHOLMODContextManager = powsybl::ContextManager<GaussNewtonCHOLMODContext>;

std::unique_ptr<GaussNewtonCHOLMODContextManager> CHOLMOD_MANAGER(new GaussNewtonCHOLMODContextManager("CHOLMOD "));

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Class:     com_powsybl_math_solver_GaussNewtonCholesky
 * Method:    init
 * Signature: (Ljava/lang/String;II[D)V
 */
JNIEXPORT void JNICALL Java_com_powsybl_math_solver_GaussNewtonCholesky_init(JNIEnv * env, jobject, jstring j_id, jint m, jint n, jdoubleArray j_w_diag) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        // Read-only: we only square the values into wSq, so skip the copy-back.
        powsybl::jni::DoubleArray w_diag(env, j_w_diag, true);

        if (m <= 0 || n <= 0) {
            throw std::runtime_error("init: m and n must be strictly positive; got m="
                                     + std::to_string(m) + ", n=" + std::to_string(n));
        }
        if (static_cast<int>(w_diag.length()) < m) {
            throw std::runtime_error("init: sqrtWeights has length " + std::to_string(w_diag.length())
                                     + " but m=" + std::to_string(m) + " weights are required");
        }

        GaussNewtonCHOLMODContext& context = CHOLMOD_MANAGER->createContext(id);

        // Cholesky only needs the upper-triangular half of C (we set stype = 1
        // on the cholmod_sparse wrapper below). Tell NormalEquations to assemble
        // only those entries, which roughly halves the contribution map.
        context.normalEq.setUpperOnly(true);

        // Remembered so factorize/solve can check they were handed the same
        // problem size - wSq is sized from m and would otherwise be overrun.
        context.initM = m;
        context.initN = n;

        const double* wSqrt = w_diag.get();
        context.wSq.resize(m);
        for (int i = 0; i < m; ++i) {
            context.wSq[i] = wSqrt[i] * wSqrt[i];
        }
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
 * Method:    factorize
 * Signature: (Ljava/lang/String;IILjava/nio/IntBuffer;Ljava/nio/IntBuffer;Ljava/nio/DoubleBuffer;)I
 */
JNIEXPORT jint JNICALL Java_com_powsybl_math_solver_GaussNewtonCholesky_factorize(JNIEnv* env, jobject, jstring j_id,
                                                                                   jint m, jint n, jobject j_ap, jobject j_ai, jobject j_ax) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        auto* ap = static_cast<int*>(env->GetDirectBufferAddress(j_ap));
        auto* ai = static_cast<int*>(env->GetDirectBufferAddress(j_ai));
        auto* ax = static_cast<double*>(env->GetDirectBufferAddress(j_ax));
        if (!ap || !ai || !ax) {
            throw std::runtime_error("factorize() requires direct buffers (ByteBuffer.allocateDirect)");
        }

        GaussNewtonCHOLMODContext& context = CHOLMOD_MANAGER->findContext(id);
        return factorizeInternal(context, n, m, ap, ai, ax);
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
    return -1;
}

/*
 * Class:     com_powsybl_math_solver_GaussNewtonCholesky
 * Method:    solveFactorized
 * Signature: (Ljava/lang/String;Ljava/nio/DoubleBuffer;Ljava/nio/DoubleBuffer;)V
 */
JNIEXPORT void JNICALL Java_com_powsybl_math_solver_GaussNewtonCholesky_solveFactorized(JNIEnv* env, jobject, jstring j_id,
                                                                                         jobject j_r, jobject j_result) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        auto* r = static_cast<double*>(env->GetDirectBufferAddress(j_r));
        auto* result = static_cast<double*>(env->GetDirectBufferAddress(j_result));
        if (!r || !result) {
            throw std::runtime_error("solveFactorized() requires direct buffers (ByteBuffer.allocateDirect)");
        }

        GaussNewtonCHOLMODContext& context = CHOLMOD_MANAGER->findContext(id);
        solveInternal(context, r, result);
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
}

/*
 * Class:     com_powsybl_math_solver_GaussNewtonCholesky
 * Method:    solveFactorizedRaw
 * Signature: (Ljava/lang/String;Ljava/nio/DoubleBuffer;Ljava/nio/DoubleBuffer;)V
 */
JNIEXPORT void JNICALL Java_com_powsybl_math_solver_GaussNewtonCholesky_solveFactorizedRaw(JNIEnv* env, jobject, jstring j_id,
                                                                                            jobject j_b, jobject j_result) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        auto* b = static_cast<double*>(env->GetDirectBufferAddress(j_b));
        auto* result = static_cast<double*>(env->GetDirectBufferAddress(j_result));
        if (!b || !result) {
            throw std::runtime_error("solveFactorizedRaw() requires direct buffers (ByteBuffer.allocateDirect)");
        }

        GaussNewtonCHOLMODContext& context = CHOLMOD_MANAGER->findContext(id);
        solveRawInternal(context, b, result);
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
}

/*
 * Class:     com_powsybl_math_solver_GaussNewtonCholesky
 * Method:    solve
 * Signature: (Ljava/lang/String;Ljava/nio/DoubleBuffer;IILjava/nio/IntBuffer;Ljava/nio/IntBuffer;Ljava/nio/DoubleBuffer;Ljava/nio/DoubleBuffer;)V
 */
JNIEXPORT void JNICALL Java_com_powsybl_math_solver_GaussNewtonCholesky_solve(JNIEnv* env, jobject, jstring j_id, jobject j_r,
                                                                              jint m, jint n, jobject j_ap, jobject j_ai, jobject j_ax, jobject j_result) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        auto* r = static_cast<double*>(env->GetDirectBufferAddress(j_r));
        auto* ap = static_cast<int*>(env->GetDirectBufferAddress(j_ap));
        auto* ai = static_cast<int*>(env->GetDirectBufferAddress(j_ai));
        auto* ax = static_cast<double*>(env->GetDirectBufferAddress(j_ax));
        auto* result = static_cast<double*>(env->GetDirectBufferAddress(j_result));
        if (!r || !ap || !ai || !ax || !result) {
            throw std::runtime_error("solve() requires direct buffers (ByteBuffer.allocateDirect)");
        }

        GaussNewtonCHOLMODContext& context = CHOLMOD_MANAGER->findContext(id);

        int rank = factorizeInternal(context, n, m, ap, ai, ax);
        if (rank != n) {
            throw std::runtime_error("solve: rank-deficient (rank=" + std::to_string(rank)
                                     + " < " + std::to_string(n) + ")");
        }
        solveInternal(context, r, result);
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
}

/*
 * Class:     com_powsybl_math_solver_GaussNewtonCholesky
 * Method:    factorizeLM
 * Signature: (Ljava/lang/String;IILjava/nio/IntBuffer;Ljava/nio/IntBuffer;Ljava/nio/DoubleBuffer;DI)I
 */
JNIEXPORT jint JNICALL Java_com_powsybl_math_solver_GaussNewtonCholesky_factorizeLM(JNIEnv* env, jobject, jstring j_id,
                                                                                    jint m, jint n,
                                                                                    jobject j_ap, jobject j_ai, jobject j_ax,
                                                                                    jdouble lambda, jint mode) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        auto* ap = static_cast<int*>(env->GetDirectBufferAddress(j_ap));
        auto* ai = static_cast<int*>(env->GetDirectBufferAddress(j_ai));
        auto* ax = static_cast<double*>(env->GetDirectBufferAddress(j_ax));
        if (!ap || !ai || !ax) {
            throw std::runtime_error("factorizeLM() requires direct buffers");
        }
        GaussNewtonCHOLMODContext& context = CHOLMOD_MANAGER->findContext(id);
        return factorizeLMInternal(context, n, m, ap, ai, ax, lambda, mode);
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
    return -1;
}

/*
 * Class:     com_powsybl_math_solver_GaussNewtonCholesky
 * Method:    refactorizeLM
 * Signature: (Ljava/lang/String;DI)I
 */
JNIEXPORT jint JNICALL Java_com_powsybl_math_solver_GaussNewtonCholesky_refactorizeLM(JNIEnv* env, jobject, jstring j_id,
                                                                                      jdouble lambda, jint mode) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        GaussNewtonCHOLMODContext& context = CHOLMOD_MANAGER->findContext(id);
        return refactorizeLMInternal(context, lambda, mode);
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
    return -1;
}

/*
 * Class:     com_powsybl_math_solver_GaussNewtonCholesky
 * Method:    solveLM
 * Signature: (Ljava/lang/String;Ljava/nio/DoubleBuffer;IILjava/nio/IntBuffer;Ljava/nio/IntBuffer;Ljava/nio/DoubleBuffer;DILjava/nio/DoubleBuffer;)V
 */
JNIEXPORT void JNICALL Java_com_powsybl_math_solver_GaussNewtonCholesky_solveLM(JNIEnv* env, jobject, jstring j_id, jobject j_r,
                                                                                 jint m, jint n,
                                                                                 jobject j_ap, jobject j_ai, jobject j_ax,
                                                                                 jdouble lambda, jint mode, jobject j_result) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        auto* r = static_cast<double*>(env->GetDirectBufferAddress(j_r));
        auto* ap = static_cast<int*>(env->GetDirectBufferAddress(j_ap));
        auto* ai = static_cast<int*>(env->GetDirectBufferAddress(j_ai));
        auto* ax = static_cast<double*>(env->GetDirectBufferAddress(j_ax));
        auto* result = static_cast<double*>(env->GetDirectBufferAddress(j_result));
        if (!r || !ap || !ai || !ax || !result) {
            throw std::runtime_error("solveLM() requires direct buffers");
        }
        GaussNewtonCHOLMODContext& context = CHOLMOD_MANAGER->findContext(id);

        int rank = factorizeLMInternal(context, n, m, ap, ai, ax, lambda, mode);
        if (rank != n) {
            throw std::runtime_error("solveLM: damped factor still rank-deficient (rank="
                                     + std::to_string(rank) + " < " + std::to_string(n)
                                     + "); increase lambda");
        }
        solveInternal(context, r, result);
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
}

#ifdef __cplusplus
}
#endif
