/**
 * Copyright (c) 2026, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * @file gaussnewton_klu.cpp
 * @author Gautier Bureau <gautier.bureau at rte-france.com>
 */

#include <string>
#include <map>
#include <memory>
#include <mutex>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <stdexcept>
#include <chrono>
#include <cstdio>
#include <klu.h>
#include "jniwrapper.hpp"
#include "gaussnewton_common.hpp"

namespace {
bool profileEnabled() {
    static const bool enabled = std::getenv("GN_PROFILE") != nullptr;
    return enabled;
}
}  // namespace

// rgrowth threshold below which we fall back to a full klu_factor instead of klu_refactor.
static constexpr double KLU_RGROWTH_THRESHOLD = 1e-3;

class GaussNewtonKLUContext {
public:
    GaussNewtonKLUContext()
        : symbolic(nullptr), numeric(nullptr) {
        klu_defaults(&common);
    }

    GaussNewtonKLUContext(const GaussNewtonKLUContext&) = delete;

    ~GaussNewtonKLUContext() {
        if (numeric) {
            klu_free_numeric(&numeric, &common);
        }
        if (symbolic) {
            klu_free_symbolic(&symbolic, &common);
        }
    }

    GaussNewtonKLUContext& operator=(const GaussNewtonKLUContext&) = delete;

    std::string error() const;

public:
    klu_common common;
    klu_symbolic* symbolic;
    klu_numeric* numeric;

    powsybl::gaussnewton::NormalEquations normalEq;
    std::vector<double> wSq;  // squared weights (length m), precomputed in init

    // -1 = no factor cached, n = full rank, k < n = first rank-deficient column.
    int lastRank = -1;
};

std::string GaussNewtonKLUContext::error() const {
    switch (common.status) {
        case KLU_OK: return "KLU_OK";
        case KLU_SINGULAR: return "KLU_SINGULAR";
        case KLU_OUT_OF_MEMORY: return "KLU_OUT_OF_MEMORY";
        case KLU_INVALID: return "KLU_INVALID";
        case KLU_TOO_LARGE: return "KLU_TOO_LARGE";
        default: return "Unknown KLU status: " + std::to_string(common.status);
    }
}

class GaussNewtonKLUContextManager {
public:
    GaussNewtonKLUContextManager() = default;

    GaussNewtonKLUContextManager(const GaussNewtonKLUContextManager&) = delete;

    ~GaussNewtonKLUContextManager() = default;

    GaussNewtonKLUContextManager& operator=(const GaussNewtonKLUContextManager&) = delete;

    GaussNewtonKLUContext& createContext(const std::string& id);

    GaussNewtonKLUContext& findContext(const std::string& id);

    void removeContext(const std::string& id);

private:
    std::map<std::string, std::unique_ptr<GaussNewtonKLUContext>> _contexts;
    std::mutex _mutex;
};

GaussNewtonKLUContext& GaussNewtonKLUContextManager::createContext(const std::string& id) {
    std::lock_guard<std::mutex> lk(_mutex);
    if (_contexts.find(id) != _contexts.end()) {
        throw std::runtime_error("KLU Context " + id + " already exists");
    }
    std::unique_ptr<GaussNewtonKLUContext> context(new GaussNewtonKLUContext());
    auto it = _contexts.insert(std::make_pair(id, std::move(context)));
    return *it.first->second;
}

GaussNewtonKLUContext& GaussNewtonKLUContextManager::findContext(const std::string& id) {
    std::lock_guard<std::mutex> lk(_mutex);
    auto it = _contexts.find(id);
    if (it == _contexts.end()) {
        throw std::runtime_error("KLU Context " + id + " not found");
    }
    return *it->second;
}

void GaussNewtonKLUContextManager::removeContext(const std::string& id) {
    std::lock_guard<std::mutex> lk(_mutex);
    _contexts.erase(id);
}

std::unique_ptr<GaussNewtonKLUContextManager> KLU_GN_MANAGER(new GaussNewtonKLUContextManager());

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Class:     com_powsybl_math_solver_GaussNewtonKLU
 * Method:    init
 * Signature: (Ljava/lang/String;II[D)V
 */
JNIEXPORT void JNICALL Java_com_powsybl_math_solver_GaussNewtonKLU_init(JNIEnv* env, jobject, jstring j_id, jint m, jint /*n*/, jdoubleArray j_w_diag) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        powsybl::jni::DoubleArray w_diag(env, j_w_diag);

        GaussNewtonKLUContext& context = KLU_GN_MANAGER->createContext(id);

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
 * Class:     com_powsybl_math_solver_GaussNewtonKLU
 * Method:    release
 * Signature: (Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_com_powsybl_math_solver_GaussNewtonKLU_release(JNIEnv* env, jobject, jstring j_id) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();

        KLU_GN_MANAGER->removeContext(id);
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
}

static powsybl::gaussnewton::NormalEquations::DampingMode mapDampingMode(jint mode) {
    using DM = powsybl::gaussnewton::NormalEquations::DampingMode;
    switch (mode) {
        case 0: return DM::IDENTITY;
        case 1: return DM::MARQUARDT;
        default:
            throw std::runtime_error("LM damping mode must be 0 (IDENTITY) or 1 (MARQUARDT); got "
                                     + std::to_string(mode));
    }
}

// Runs KLU analyze (lazy, once) + factor/refactor on the current cx_. Sets
// context.lastRank to n on success, or to common.singular_col on KLU_SINGULAR.
static int factorCachedC(GaussNewtonKLUContext& context, int n,
                         const char** outPath = nullptr) {
    int* Cp = context.normalEq.cp();
    int* Ci = context.normalEq.ci();
    double* Cx = context.normalEq.cx();

    if (!context.symbolic) {
        context.symbolic = klu_analyze(n, Cp, Ci, &context.common);
        if (!context.symbolic) {
            throw std::runtime_error("klu_analyze error " + context.error());
        }
    }

    const char* factorPath = "factor";
    int rank = n;
    auto handleSingularOrError = [&](const char* opName) {
        if (context.common.status == KLU_SINGULAR) {
            rank = static_cast<int>(context.common.singular_col);
            if (context.numeric) {
                klu_free_numeric(&context.numeric, &context.common);
            }
        } else {
            throw std::runtime_error(std::string(opName) + " error " + context.error());
        }
    };

    if (!context.numeric) {
        context.numeric = klu_factor(Cp, Ci, Cx, context.symbolic, &context.common);
        if (!context.numeric) {
            handleSingularOrError("klu_factor");
        }
    } else {
        int ok = klu_refactor(Cp, Ci, Cx, context.symbolic, context.numeric, &context.common);
        if (ok == 0) {
            handleSingularOrError("klu_refactor");
        } else {
            ok = klu_rgrowth(Cp, Ci, Cx, context.symbolic, context.numeric, &context.common);
            if (ok == 0) {
                throw std::runtime_error("klu_rgrowth error " + context.error());
            }
            if (context.common.rgrowth < KLU_RGROWTH_THRESHOLD) {
                if (klu_free_numeric(&context.numeric, &context.common) == 0) {
                    throw std::runtime_error("klu_free_numeric error " + context.error());
                }
                context.numeric = klu_factor(Cp, Ci, Cx, context.symbolic, &context.common);
                if (!context.numeric) {
                    handleSingularOrError("klu_factor");
                }
                factorPath = "refactor->factor";
            } else {
                factorPath = "refactor";
            }
        }
    }
    context.lastRank = rank;
    if (outPath) {
        *outPath = factorPath;
    }
    return rank;
}

// GN factorize: assemble C, factor as-is.
static int factorizeInternal(GaussNewtonKLUContext& context, int n, int m,
                      const int* ap, const int* ai, const double* ax,
                      double* outUpdateMs = nullptr, double* outFactorMs = nullptr,
                      const char** outPath = nullptr) {
    const bool profile = (outUpdateMs != nullptr);
    auto t0 = profile ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};

    context.normalEq.updateMatrix(n, m, ap, ai, ax, context.wSq.data());
    auto t1 = profile ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};

    int rank = factorCachedC(context, n, outPath);
    auto t2 = profile ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};

    if (profile) {
        auto ms = [](auto a, auto b) {
            return std::chrono::duration<double, std::milli>(b - a).count();
        };
        *outUpdateMs = ms(t0, t1);
        *outFactorMs = ms(t1, t2);
    }
    return rank;
}

// LM factorize: assemble C, apply damping, factor.
static int factorizeLMInternal(GaussNewtonKLUContext& context, int n, int m,
                               const int* ap, const int* ai, const double* ax,
                               double lambda, jint mode,
                               double* outUpdateMs = nullptr,
                               double* outDampMs = nullptr,
                               double* outFactorMs = nullptr,
                               const char** outPath = nullptr) {
    const bool profile = (outUpdateMs != nullptr);
    auto t0 = profile ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};

    context.normalEq.updateMatrix(n, m, ap, ai, ax, context.wSq.data());
    auto t1 = profile ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};

    context.normalEq.applyDamping(lambda, mapDampingMode(mode));
    auto t2 = profile ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};

    int rank = factorCachedC(context, n, outPath);
    auto t3 = profile ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};

    if (profile) {
        auto ms = [](auto a, auto b) {
            return std::chrono::duration<double, std::milli>(b - a).count();
        };
        *outUpdateMs = ms(t0, t1);
        *outDampMs = ms(t1, t2);
        *outFactorMs = ms(t2, t3);
    }
    return rank;
}

// LM refactorize: reuse cached Ht/C, apply damping with a new lambda, factor.
static int refactorizeLMInternal(GaussNewtonKLUContext& context,
                                 double lambda, jint mode,
                                 double* outDampMs = nullptr,
                                 double* outFactorMs = nullptr,
                                 const char** outPath = nullptr) {
    if (!context.normalEq.patternBuilt()) {
        throw std::runtime_error("refactorizeLM: no pattern cached; call factorize/factorizeLM first");
    }
    const int n = context.normalEq.n();

    const bool profile = (outDampMs != nullptr);
    auto t0 = profile ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};

    context.normalEq.applyDamping(lambda, mapDampingMode(mode));
    auto t1 = profile ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};

    int rank = factorCachedC(context, n, outPath);
    auto t2 = profile ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};

    if (profile) {
        auto ms = [](auto a, auto b) {
            return std::chrono::duration<double, std::milli>(b - a).count();
        };
        *outDampMs = ms(t0, t1);
        *outFactorMs = ms(t1, t2);
    }
    return rank;
}

static void solveInternal(GaussNewtonKLUContext& context, const double* r, double* result,
                   double* outRhsMs = nullptr, double* outSolveMs = nullptr) {
    if (context.lastRank < 0) {
        throw std::runtime_error("solveFactorized: no factor cached (call factorize first)");
    }
    const int n = context.normalEq.n();
    if (context.lastRank != n) {
        throw std::runtime_error("solveFactorized: last factorize was rank-deficient (rank="
                                 + std::to_string(context.lastRank) + " < " + std::to_string(n) + ")");
    }

    const bool profile = (outRhsMs != nullptr);
    auto t0 = profile ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};

    context.normalEq.computeRhs(context.wSq.data(), r);
    auto t1 = profile ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};

    std::memcpy(result, context.normalEq.rhs(), n * sizeof(double));
    if (klu_solve(context.symbolic, context.numeric, n, 1, result, &context.common) == 0) {
        throw std::runtime_error("klu_solve error " + context.error());
    }
    auto t2 = profile ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};

    if (profile) {
        auto ms = [](auto a, auto b) {
            return std::chrono::duration<double, std::milli>(b - a).count();
        };
        *outRhsMs = ms(t0, t1);
        *outSolveMs = ms(t1, t2);
    }
}

/**
 * Solve C * x = b directly, skipping the H'W r right-hand-side assembly. Used
 * when the caller already has a fully-formed n-vector b (e.g. a unit vector
 * for state-covariance columns of C^-1). Same rank guards as solveInternal.
 */
static void solveRawInternal(GaussNewtonKLUContext& context, const double* b, double* result,
                              double* outSolveMs = nullptr) {
    if (context.lastRank < 0) {
        throw std::runtime_error("solveFactorizedRaw: no factor cached (call factorize first)");
    }
    const int n = context.normalEq.n();
    if (context.lastRank != n) {
        throw std::runtime_error("solveFactorizedRaw: last factorize was rank-deficient (rank="
                                 + std::to_string(context.lastRank) + " < " + std::to_string(n) + ")");
    }

    const bool profile = (outSolveMs != nullptr);
    auto t0 = profile ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};

    // klu_solve overwrites its rhs buffer with the solution.
    std::memcpy(result, b, n * sizeof(double));
    if (klu_solve(context.symbolic, context.numeric, n, 1, result, &context.common) == 0) {
        throw std::runtime_error("klu_solve error " + context.error());
    }

    if (profile) {
        auto t1 = std::chrono::steady_clock::now();
        *outSolveMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    }
}

/*
 * Class:     com_powsybl_math_solver_GaussNewtonKLU
 * Method:    factorize
 * Signature: (Ljava/lang/String;IILjava/nio/IntBuffer;Ljava/nio/IntBuffer;Ljava/nio/DoubleBuffer;)I
 */
JNIEXPORT jint JNICALL Java_com_powsybl_math_solver_GaussNewtonKLU_factorize(JNIEnv* env, jobject, jstring j_id,
                                                                              jint m, jint n, jobject j_ap, jobject j_ai, jobject j_ax) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        auto* ap = static_cast<int*>(env->GetDirectBufferAddress(j_ap));
        auto* ai = static_cast<int*>(env->GetDirectBufferAddress(j_ai));
        auto* ax = static_cast<double*>(env->GetDirectBufferAddress(j_ax));
        if (!ap || !ai || !ax) {
            throw std::runtime_error("factorize() requires direct buffers (ByteBuffer.allocateDirect)");
        }

        GaussNewtonKLUContext& context = KLU_GN_MANAGER->findContext(id);

        if (profileEnabled()) {
            double tUpdate = 0.0;
            double tFactor = 0.0;
            const char* path = "factor";
            int rank = factorizeInternal(context, n, m, ap, ai, ax, &tUpdate, &tFactor, &path);
            std::fprintf(stderr, "[gn-klu factorize] update=%.3f factorize[%s,rgrowth=%.2e]=%.3f ms rank=%d\n",
                         tUpdate, path, context.common.rgrowth, tFactor, rank);
            return rank;
        }
        return factorizeInternal(context, n, m, ap, ai, ax);
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
    return -1;
}

/*
 * Class:     com_powsybl_math_solver_GaussNewtonKLU
 * Method:    solveFactorized
 * Signature: (Ljava/lang/String;Ljava/nio/DoubleBuffer;Ljava/nio/DoubleBuffer;)V
 */
JNIEXPORT void JNICALL Java_com_powsybl_math_solver_GaussNewtonKLU_solveFactorized(JNIEnv* env, jobject, jstring j_id,
                                                                                    jobject j_r, jobject j_result) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        auto* r = static_cast<double*>(env->GetDirectBufferAddress(j_r));
        auto* result = static_cast<double*>(env->GetDirectBufferAddress(j_result));
        if (!r || !result) {
            throw std::runtime_error("solveFactorized() requires direct buffers (ByteBuffer.allocateDirect)");
        }

        GaussNewtonKLUContext& context = KLU_GN_MANAGER->findContext(id);

        if (profileEnabled()) {
            double tRhs = 0.0;
            double tSolve = 0.0;
            solveInternal(context, r, result, &tRhs, &tSolve);
            std::fprintf(stderr, "[gn-klu solveFactorized] rhs=%.3f solve=%.3f ms\n", tRhs, tSolve);
        } else {
            solveInternal(context, r, result);
        }
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
}

/*
 * Class:     com_powsybl_math_solver_GaussNewtonKLU
 * Method:    solveFactorizedRaw
 * Signature: (Ljava/lang/String;Ljava/nio/DoubleBuffer;Ljava/nio/DoubleBuffer;)V
 */
JNIEXPORT void JNICALL Java_com_powsybl_math_solver_GaussNewtonKLU_solveFactorizedRaw(JNIEnv* env, jobject, jstring j_id,
                                                                                       jobject j_b, jobject j_result) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        auto* b = static_cast<double*>(env->GetDirectBufferAddress(j_b));
        auto* result = static_cast<double*>(env->GetDirectBufferAddress(j_result));
        if (!b || !result) {
            throw std::runtime_error("solveFactorizedRaw() requires direct buffers (ByteBuffer.allocateDirect)");
        }

        GaussNewtonKLUContext& context = KLU_GN_MANAGER->findContext(id);

        if (profileEnabled()) {
            double tSolve = 0.0;
            solveRawInternal(context, b, result, &tSolve);
            std::fprintf(stderr, "[gn-klu solveFactorizedRaw] solve=%.3f ms\n", tSolve);
        } else {
            solveRawInternal(context, b, result);
        }
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
}

/*
 * Class:     com_powsybl_math_solver_GaussNewtonKLU
 * Method:    solve
 * Signature: (Ljava/lang/String;Ljava/nio/DoubleBuffer;IILjava/nio/IntBuffer;Ljava/nio/IntBuffer;Ljava/nio/DoubleBuffer;Ljava/nio/DoubleBuffer;)V
 */
JNIEXPORT void JNICALL Java_com_powsybl_math_solver_GaussNewtonKLU_solve(JNIEnv* env, jobject, jstring j_id, jobject j_r,
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

        GaussNewtonKLUContext& context = KLU_GN_MANAGER->findContext(id);

        const bool profile = profileEnabled();
        double tUpdate = 0.0;
        double tFactor = 0.0;
        double tRhs = 0.0;
        double tSolve = 0.0;
        const char* path = "factor";
        int rank = profile
                ? factorizeInternal(context, n, m, ap, ai, ax, &tUpdate, &tFactor, &path)
                : factorizeInternal(context, n, m, ap, ai, ax);
        if (rank != n) {
            throw std::runtime_error("solve: rank-deficient (rank=" + std::to_string(rank)
                                     + " < " + std::to_string(n) + ")");
        }
        if (profile) {
            solveInternal(context, r, result, &tRhs, &tSolve);
            std::fprintf(stderr, "[gn-klu] update=%.3f factorize[%s,rgrowth=%.2e]=%.3f rhs=%.3f solve=%.3f ms\n",
                         tUpdate, path, context.common.rgrowth, tFactor, tRhs, tSolve);
        } else {
            solveInternal(context, r, result);
        }
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
}

/*
 * Class:     com_powsybl_math_solver_GaussNewtonKLU
 * Method:    factorizeLM
 * Signature: (Ljava/lang/String;IILjava/nio/IntBuffer;Ljava/nio/IntBuffer;Ljava/nio/DoubleBuffer;DI)I
 */
JNIEXPORT jint JNICALL Java_com_powsybl_math_solver_GaussNewtonKLU_factorizeLM(JNIEnv* env, jobject, jstring j_id,
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
        GaussNewtonKLUContext& context = KLU_GN_MANAGER->findContext(id);

        if (profileEnabled()) {
            double tUpdate = 0.0;
            double tDamp = 0.0;
            double tFactor = 0.0;
            const char* path = "factor";
            int rank = factorizeLMInternal(context, n, m, ap, ai, ax, lambda, mode,
                                           &tUpdate, &tDamp, &tFactor, &path);
            std::fprintf(stderr, "[gn-klu factorizeLM lambda=%.3e mode=%d] update=%.3f damp=%.3f factorize[%s,rgrowth=%.2e]=%.3f ms rank=%d\n",
                         lambda, mode, tUpdate, tDamp, path, context.common.rgrowth, tFactor, rank);
            return rank;
        }
        return factorizeLMInternal(context, n, m, ap, ai, ax, lambda, mode);
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
    return -1;
}

/*
 * Class:     com_powsybl_math_solver_GaussNewtonKLU
 * Method:    refactorizeLM
 * Signature: (Ljava/lang/String;DI)I
 */
JNIEXPORT jint JNICALL Java_com_powsybl_math_solver_GaussNewtonKLU_refactorizeLM(JNIEnv* env, jobject, jstring j_id,
                                                                                  jdouble lambda, jint mode) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        GaussNewtonKLUContext& context = KLU_GN_MANAGER->findContext(id);

        if (profileEnabled()) {
            double tDamp = 0.0;
            double tFactor = 0.0;
            const char* path = "factor";
            int rank = refactorizeLMInternal(context, lambda, mode, &tDamp, &tFactor, &path);
            std::fprintf(stderr, "[gn-klu refactorizeLM lambda=%.3e mode=%d] damp=%.3f factorize[%s,rgrowth=%.2e]=%.3f ms rank=%d\n",
                         lambda, mode, tDamp, path, context.common.rgrowth, tFactor, rank);
            return rank;
        }
        return refactorizeLMInternal(context, lambda, mode);
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
    return -1;
}

/*
 * Class:     com_powsybl_math_solver_GaussNewtonKLU
 * Method:    solveLM
 * Signature: (Ljava/lang/String;Ljava/nio/DoubleBuffer;IILjava/nio/IntBuffer;Ljava/nio/IntBuffer;Ljava/nio/DoubleBuffer;DILjava/nio/DoubleBuffer;)V
 */
JNIEXPORT void JNICALL Java_com_powsybl_math_solver_GaussNewtonKLU_solveLM(JNIEnv* env, jobject, jstring j_id, jobject j_r,
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
        GaussNewtonKLUContext& context = KLU_GN_MANAGER->findContext(id);

        const bool profile = profileEnabled();
        double tUpdate = 0.0;
        double tDamp = 0.0;
        double tFactor = 0.0;
        double tRhs = 0.0;
        double tSolve = 0.0;
        const char* path = "factor";
        int rank = profile
                ? factorizeLMInternal(context, n, m, ap, ai, ax, lambda, mode,
                                      &tUpdate, &tDamp, &tFactor, &path)
                : factorizeLMInternal(context, n, m, ap, ai, ax, lambda, mode);
        if (rank != n) {
            throw std::runtime_error("solveLM: damped factor still rank-deficient (rank="
                                     + std::to_string(rank) + " < " + std::to_string(n)
                                     + "); increase lambda");
        }
        if (profile) {
            solveInternal(context, r, result, &tRhs, &tSolve);
            std::fprintf(stderr, "[gn-klu solveLM lambda=%.3e mode=%d] update=%.3f damp=%.3f factorize[%s,rgrowth=%.2e]=%.3f rhs=%.3f solve=%.3f ms\n",
                         lambda, mode, tUpdate, tDamp, path, context.common.rgrowth, tFactor, tRhs, tSolve);
        } else {
            solveInternal(context, r, result);
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
