/**
 * Copyright (c) 2026, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 *
 * @file cudss.cpp
 *
 * JNI bindings for an LU decomposition backed by NVIDIA cuDSS, mirroring the
 * KLU implementation in lu.cpp. Built into a SEPARATE shared library
 * (libmathcudss) so the CUDA dependency never touches the CPU-only libmath.
 *
 * Matrix convention (see docs/cudss-integration.md section 6): the (ap, ai, ax)
 * arrays are the CSC of a matrix M (powsybl SparseMatrix layout). They are fed
 * to cuDSS AS CSR, so cuDSS factorizes N = M^T. A plain cuDSS solve therefore
 * computes M^T x = b, i.e. solve(transpose=true) == solveTransposed, which is
 * the path open-loadflow uses. cuDSS has no transpose solve, so the (unused on
 * sparse) non-transposed solve throws.
 */
#include <string>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <cuda_runtime.h>
#include <cudss.h>
#include "jniwrapper.hpp"

#define CUDA_CHECK(x) do { cudaError_t e_ = (x); if (e_ != cudaSuccess) { \
    throw std::runtime_error(std::string("CUDA error: ") + cudaGetErrorString(e_)); } } while (0)
#define CUDSS_CHECK(x) do { cudssStatus_t s_ = (x); if (s_ != CUDSS_STATUS_SUCCESS) { \
    throw std::runtime_error("cuDSS error, status " + std::to_string((int) s_)); } } while (0)

namespace {

// Number of iterative-refinement steps. cuDSS default pivoting leaves ~1e-3
// error on unsymmetric systems; 2 steps recover KLU-grade accuracy.
constexpr int IR_N_STEPS = 2;

class CuDssContext {
public:
    CuDssContext() = default;
    CuDssContext(const CuDssContext&) = delete;
    CuDssContext& operator=(const CuDssContext&) = delete;
    ~CuDssContext() { destroy(); }

    // ensure the rhs/solution device buffers and dense matrices match (rows, cols)
    void ensureRhs(int rows, int cols) {
        int needed = rows * cols;
        if (needed > rhsCapacity) {
            if (d_b) { cudaFree(d_b); d_b = nullptr; }
            if (d_x) { cudaFree(d_x); d_x = nullptr; }
            CUDA_CHECK(cudaMalloc(&d_b, needed * sizeof(double)));
            CUDA_CHECK(cudaMalloc(&d_x, needed * sizeof(double)));
            rhsCapacity = needed;
            // buffers moved, force matrix recreation
            if (matB) { cudssMatrixDestroy(matB); matB = nullptr; }
            if (matX) { cudssMatrixDestroy(matX); matX = nullptr; }
        }
        if (matB == nullptr || rhsRows != rows || rhsCols != cols) {
            if (matB) { cudssMatrixDestroy(matB); matB = nullptr; }
            if (matX) { cudssMatrixDestroy(matX); matX = nullptr; }
            CUDSS_CHECK(cudssMatrixCreateDn(&matB, rows, cols, rows, d_b, CUDSS_R_64F, CUDSS_LAYOUT_COL_MAJOR));
            CUDSS_CHECK(cudssMatrixCreateDn(&matX, rows, cols, rows, d_x, CUDSS_R_64F, CUDSS_LAYOUT_COL_MAJOR));
            rhsRows = rows;
            rhsCols = cols;
        }
    }

    void destroy() {
        if (matA) { cudssMatrixDestroy(matA); matA = nullptr; }
        if (matB) { cudssMatrixDestroy(matB); matB = nullptr; }
        if (matX) { cudssMatrixDestroy(matX); matX = nullptr; }
        if (data && handle) { cudssDataDestroy(handle, data); data = nullptr; }
        if (config) { cudssConfigDestroy(config); config = nullptr; }
        if (handle) { cudssDestroy(handle); handle = nullptr; }
        if (stream) { cudaStreamDestroy(stream); stream = nullptr; }
        if (d_ap) { cudaFree(d_ap); d_ap = nullptr; }
        if (d_ai) { cudaFree(d_ai); d_ai = nullptr; }
        if (d_ax) { cudaFree(d_ax); d_ax = nullptr; }
        if (d_b) { cudaFree(d_b); d_b = nullptr; }
        if (d_x) { cudaFree(d_x); d_x = nullptr; }
    }

    cudaStream_t stream = nullptr;
    cudssHandle_t handle = nullptr;
    cudssConfig_t config = nullptr;
    cudssData_t data = nullptr;
    cudssMatrix_t matA = nullptr;
    cudssMatrix_t matB = nullptr;
    cudssMatrix_t matX = nullptr;
    int* d_ap = nullptr;
    int* d_ai = nullptr;
    double* d_ax = nullptr;
    double* d_b = nullptr;
    double* d_x = nullptr;
    int n = 0;
    int nnz = 0;
    int rhsCapacity = 0;
    int rhsRows = 0;
    int rhsCols = 0;
};

class CuDssContextManager {
public:
    CuDssContext& createContext(const std::string& id) {
        std::lock_guard<std::mutex> lk(_mutex);
        if (_contexts.find(id) != _contexts.end()) {
            throw std::runtime_error("Context " + id + " already exists");
        }
        auto it = _contexts.insert(std::make_pair(id, std::unique_ptr<CuDssContext>(new CuDssContext())));
        return *it.first->second;
    }

    CuDssContext& findContext(const std::string& id) {
        std::lock_guard<std::mutex> lk(_mutex);
        auto it = _contexts.find(id);
        if (it == _contexts.end()) {
            throw std::runtime_error("Context " + id + " not found");
        }
        return *it->second;
    }

    void removeContext(const std::string& id) {
        std::lock_guard<std::mutex> lk(_mutex);
        _contexts.erase(id);
    }

private:
    std::map<std::string, std::unique_ptr<CuDssContext>> _contexts;
    std::mutex _mutex;
};

std::unique_ptr<CuDssContextManager> MANAGER(new CuDssContextManager());

void solveInto(CuDssContext& ctx, double* host, int rows, int cols, bool transpose) {
    if (!transpose) {
        throw std::runtime_error("cuDSS backend supports only the transposed solve "
                                 "(non-transposed sparse solve is not implemented)");
    }
    ctx.ensureRhs(rows, cols);
    int total = rows * cols;
    CUDA_CHECK(cudaMemcpyAsync(ctx.d_b, host, total * sizeof(double), cudaMemcpyHostToDevice, ctx.stream));
    CUDSS_CHECK(cudssExecute(ctx.handle, CUDSS_PHASE_SOLVE, ctx.config, ctx.data, ctx.matA, ctx.matX, ctx.matB));
    CUDA_CHECK(cudaMemcpyAsync(host, ctx.d_x, total * sizeof(double), cudaMemcpyDeviceToHost, ctx.stream));
    CUDA_CHECK(cudaStreamSynchronize(ctx.stream));
}

}  // namespace

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Class:     com_powsybl_math_matrix_CuDssLUDecomposition
 * Method:    init
 * Signature: (Ljava/lang/String;[I[I[D)V
 */
JNIEXPORT void JNICALL Java_com_powsybl_math_matrix_CuDssLUDecomposition_init(JNIEnv* env, jobject, jstring j_id, jintArray j_ap, jintArray j_ai, jdoubleArray j_ax) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        powsybl::jni::IntArray ap(env, j_ap);
        powsybl::jni::IntArray ai(env, j_ai);
        powsybl::jni::DoubleArray ax(env, j_ax);

        CuDssContext& ctx = MANAGER->createContext(id);
        ctx.n = static_cast<int>(ap.length()) - 1;
        ctx.nnz = static_cast<int>(ax.length());

        CUDA_CHECK(cudaStreamCreate(&ctx.stream));
        CUDSS_CHECK(cudssCreate(&ctx.handle));
        CUDSS_CHECK(cudssSetStream(ctx.handle, ctx.stream));
        CUDSS_CHECK(cudssConfigCreate(&ctx.config));
        int irSteps = IR_N_STEPS;
        CUDSS_CHECK(cudssConfigSet(ctx.config, CUDSS_CONFIG_IR_N_STEPS, &irSteps, sizeof(irSteps)));
        CUDSS_CHECK(cudssDataCreate(ctx.handle, &ctx.data));

        CUDA_CHECK(cudaMalloc(&ctx.d_ap, (ctx.n + 1) * sizeof(int)));
        CUDA_CHECK(cudaMalloc(&ctx.d_ai, ctx.nnz * sizeof(int)));
        CUDA_CHECK(cudaMalloc(&ctx.d_ax, ctx.nnz * sizeof(double)));
        CUDA_CHECK(cudaMemcpyAsync(ctx.d_ap, ap.get(), (ctx.n + 1) * sizeof(int), cudaMemcpyHostToDevice, ctx.stream));
        CUDA_CHECK(cudaMemcpyAsync(ctx.d_ai, ai.get(), ctx.nnz * sizeof(int), cudaMemcpyHostToDevice, ctx.stream));
        CUDA_CHECK(cudaMemcpyAsync(ctx.d_ax, ax.get(), ctx.nnz * sizeof(double), cudaMemcpyHostToDevice, ctx.stream));

        // CSC of M passed as CSR => cuDSS factorizes N = M^T (see file header).
        CUDSS_CHECK(cudssMatrixCreateCsr(&ctx.matA, ctx.n, ctx.n, ctx.nnz,
                                         ctx.d_ap, nullptr, ctx.d_ai, ctx.d_ax,
                                         CUDSS_R_32I, CUDSS_R_32I, CUDSS_R_64F,
                                         CUDSS_MTYPE_GENERAL, CUDSS_MVIEW_FULL, CUDSS_BASE_ZERO));
        ctx.ensureRhs(ctx.n, 1);

        CUDSS_CHECK(cudssExecute(ctx.handle, CUDSS_PHASE_ANALYSIS, ctx.config, ctx.data, ctx.matA, ctx.matX, ctx.matB));
        CUDSS_CHECK(cudssExecute(ctx.handle, CUDSS_PHASE_FACTORIZATION, ctx.config, ctx.data, ctx.matA, ctx.matX, ctx.matB));
        CUDA_CHECK(cudaStreamSynchronize(ctx.stream));
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
}

/*
 * Class:     com_powsybl_math_matrix_CuDssLUDecomposition
 * Method:    update
 * Signature: (Ljava/lang/String;[I[I[DD)D
 */
JNIEXPORT jdouble JNICALL Java_com_powsybl_math_matrix_CuDssLUDecomposition_update(JNIEnv* env, jobject, jstring j_id, jintArray, jintArray, jdoubleArray j_ax, jdouble) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        powsybl::jni::DoubleArray ax(env, j_ax);

        CuDssContext& ctx = MANAGER->findContext(id);
        if (static_cast<int>(ax.length()) != ctx.nnz) {
            throw std::runtime_error("Matrix structure changed since initial decomposition");
        }
        // structure unchanged: refresh values in place and refactorize
        CUDA_CHECK(cudaMemcpyAsync(ctx.d_ax, ax.get(), ctx.nnz * sizeof(double), cudaMemcpyHostToDevice, ctx.stream));
        CUDSS_CHECK(cudssExecute(ctx.handle, CUDSS_PHASE_REFACTORIZATION, ctx.config, ctx.data, ctx.matA, ctx.matX, ctx.matB));
        CUDA_CHECK(cudaStreamSynchronize(ctx.stream));
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
    return 1.0;  // cuDSS has no rgrowth metric; report a benign value
}

/*
 * Class:     com_powsybl_math_matrix_CuDssLUDecomposition
 * Method:    release
 * Signature: (Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_com_powsybl_math_matrix_CuDssLUDecomposition_release(JNIEnv* env, jobject, jstring j_id) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        CuDssContext& ctx = MANAGER->findContext(id);
        ctx.destroy();
        MANAGER->removeContext(id);
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
}

/*
 * Class:     com_powsybl_math_matrix_CuDssLUDecomposition
 * Method:    solve
 * Signature: (Ljava/lang/String;[DZ)V
 */
JNIEXPORT void JNICALL Java_com_powsybl_math_matrix_CuDssLUDecomposition_solve(JNIEnv* env, jobject, jstring j_id, jdoubleArray j_b, jboolean transpose) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        powsybl::jni::DoubleArray b(env, j_b);
        CuDssContext& ctx = MANAGER->findContext(id);
        solveInto(ctx, b.get(), static_cast<int>(b.length()), 1, transpose);
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
}

/*
 * Class:     com_powsybl_math_matrix_CuDssLUDecomposition
 * Method:    solve2
 * Signature: (Ljava/lang/String;IILjava/nio/ByteBuffer;Z)V
 */
JNIEXPORT void JNICALL Java_com_powsybl_math_matrix_CuDssLUDecomposition_solve2(JNIEnv* env, jobject, jstring j_id, jint m, jint n, jobject j_b, jboolean transpose) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        auto* b = static_cast<double*>(env->GetDirectBufferAddress(j_b));
        if (!b) {
            throw std::runtime_error("GetDirectBufferAddress error");
        }
        CuDssContext& ctx = MANAGER->findContext(id);
        solveInto(ctx, b, m, n, transpose);
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
}

#ifdef __cplusplus
}
#endif
