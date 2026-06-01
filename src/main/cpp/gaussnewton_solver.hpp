/**
 * Copyright (c) 2026, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 *
 * @file gaussnewton_solver.hpp
 * @author Gautier Bureau <gautier.bureau at rte-france.com>
 */

#ifndef GAUSSNEWTON_SOLVER_HPP
#define GAUSSNEWTON_SOLVER_HPP

#include <stdexcept>
#include <string>
#include <vector>
#include <jni.h>
#include "gaussnewton_common.hpp"

namespace powsybl {

namespace gaussnewton {

/**
 * State shared by every Gauss-Newton backend context. Backends derive from
 * this, add their solver-specific handles, and implement two hooks consumed by
 * the templated orchestration below:
 *
 *   int  analyzeFactor(int n);
 *       Analyze (lazily, once) then factor the current contents of normalEq's
 *       C matrix. Must set lastRank (n on success, the first rank-deficient
 *       column otherwise) and return it.
 *
 *   void solveLinear(int n, const double* rhs, double* result);
 *       Solve C * x = rhs into result (length n), reusing the cached factor.
 */
struct ContextBase {
    NormalEquations normalEq;
    std::vector<double> wSq;  // squared weights (length initM), precomputed in init

    // Problem dimensions declared by init(). wSq is sized from initM, so every
    // later factorize/solve must present the same m or we would read past its
    // end; see checkDimensions(). -1 = init() has not run yet.
    int initM = -1;
    int initN = -1;

    // -1 = no factor cached, n = full rank, k < n = first rank-deficient column.
    int lastRank = -1;
};

/**
 * Enforce the init()/factorize() dimension contract. NormalEquations only
 * revalidates the Ht pattern across successive solves, so without this an
 * m larger than init's would silently read past the end of wSq.
 */
inline void checkDimensions(const ContextBase& context, int n, int m) {
    if (context.initM < 0) {
        throw std::runtime_error("init() must be called before factorize/solve");
    }
    if (m != context.initM || n != context.initN) {
        throw std::runtime_error("dimension mismatch: init() declared m=" + std::to_string(context.initM)
                                 + ", n=" + std::to_string(context.initN) + " but got m=" + std::to_string(m)
                                 + ", n=" + std::to_string(n) + "; call init() again to resize");
    }
}

// Map a Java mode int to the C++ enum. Single place to validate.
inline NormalEquations::DampingMode mapDampingMode(jint mode) {
    using DM = NormalEquations::DampingMode;
    switch (mode) {
        case 0: return DM::IDENTITY;
        case 1: return DM::MARQUARDT;
        default:
            throw std::runtime_error("LM damping mode must be 0 (IDENTITY) or 1 (MARQUARDT); got "
                                     + std::to_string(mode));
    }
}

// GN factorize: assemble C, factor as-is (no LM damping).
template <class Ctx>
int factorizeInternal(Ctx& context, int n, int m,
                      const int* ap, const int* ai, const double* ax) {
    checkDimensions(context, n, m);
    context.normalEq.updateMatrix(n, m, ap, ai, ax, context.wSq.data());
    return context.analyzeFactor(n);
}

// LM factorize: assemble C, apply LM damping, factor.
template <class Ctx>
int factorizeLMInternal(Ctx& context, int n, int m,
                        const int* ap, const int* ai, const double* ax,
                        double lambda, jint mode) {
    checkDimensions(context, n, m);
    context.normalEq.updateMatrix(n, m, ap, ai, ax, context.wSq.data());
    context.normalEq.applyDamping(lambda, mapDampingMode(mode));
    return context.analyzeFactor(n);
}

// LM refactorize: keep cached Ht/C contributions, re-apply damping with a new
// lambda, factor again. Used in the LM outer loop when a step was rejected and
// we want to retry with a larger lambda without rebuilding C.
template <class Ctx>
int refactorizeLMInternal(Ctx& context, double lambda, jint mode) {
    if (!context.normalEq.patternBuilt()) {
        throw std::runtime_error("refactorizeLM: no pattern cached; call factorize/factorizeLM first");
    }
    const int n = context.normalEq.n();
    context.normalEq.applyDamping(lambda, mapDampingMode(mode));
    return context.analyzeFactor(n);
}

template <class Ctx>
void solveInternal(Ctx& context, const double* r, double* result) {
    if (context.lastRank < 0) {
        throw std::runtime_error("solveFactorized: no factor cached (call factorize first)");
    }
    const int n = context.normalEq.n();
    if (context.lastRank != n) {
        throw std::runtime_error("solveFactorized: last factorize was rank-deficient (rank="
                                 + std::to_string(context.lastRank) + " < " + std::to_string(n) + ")");
    }
    context.normalEq.computeRhs(context.wSq.data(), r);
    context.solveLinear(n, context.normalEq.rhs(), result);
}

/**
 * Solve C * x = b directly, skipping the H'W r right-hand-side assembly. Used
 * when the caller already has a fully-formed n-vector b (e.g. a unit vector
 * b = e_j to extract a single column of C^-1). Same rank guards as solveInternal.
 */
template <class Ctx>
void solveRawInternal(Ctx& context, const double* b, double* result) {
    if (context.lastRank < 0) {
        throw std::runtime_error("solveFactorizedRaw: no factor cached (call factorize first)");
    }
    const int n = context.normalEq.n();
    if (context.lastRank != n) {
        throw std::runtime_error("solveFactorizedRaw: last factorize was rank-deficient (rank="
                                 + std::to_string(context.lastRank) + " < " + std::to_string(n) + ")");
    }
    context.solveLinear(n, b, result);
}

}  // namespace gaussnewton

}  // namespace powsybl

#endif // GAUSSNEWTON_SOLVER_HPP
