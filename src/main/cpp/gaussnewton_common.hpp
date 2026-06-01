/**
 * Copyright (c) 2026, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * @file gaussnewton_common.hpp
 * @author Gautier Bureau <gautier.bureau at rte-france.com>
 */

#ifndef GAUSSNEWTON_COMMON_HPP
#define GAUSSNEWTON_COMMON_HPP

#include <vector>

namespace powsybl {

namespace gaussnewton {

/**
 * Assembles the weighted normal equations C = H' W H and rhs = H' W r
 * from a column-compressed Ht (transpose of the Jacobian).
 *
 * On the first update() call the sparsity pattern of C is computed from Ht's
 * pattern and a contribution map is built. Subsequent updates re-validate the
 * pattern and only recompute Cx and rhs values - no per-solve allocation,
 * no library AAt call, no transpose.
 *
 * Layout of inputs:
 *   - Ht is (n x m) in CSC: htP (m+1), htI (nnz), htX (nnz).
 *   - wSqrt is sqrt of the diagonal weights (length m). Stable across solves.
 *   - r is the residual vector (length m).
 *
 * Outputs after update():
 *   - cp(), ci(), cx() form a CSC (n x n) full-symmetric representation of C.
 *   - rhs() is the right-hand side (length n).
 */
class NormalEquations {
public:
    /**
     * When true, only the upper-triangular part of C (entries with i <= j in
     * column j) is assembled. Suitable for symmetric solvers like CHOLMOD
     * with stype = 1. Must be set before the first update() call.
     */
    void setUpperOnly(bool v) { upperOnly_ = v; }

    /**
     * Build (or revalidate) the C-pattern and refresh Cx values. Caches the
     * supplied Ht values so computeRhs() can later rebuild the rhs without the
     * caller passing Ht again.
     */
    void updateMatrix(int n, int m,
                      const int* htP, const int* htI, const double* htX,
                      const double* wSq);

    /**
     * Compute rhs = Ht * (wSq .* r) using the htX values cached by the most
     * recent updateMatrix() call. Caller is responsible for sequencing.
     */
    void computeRhs(const double* wSq, const double* r);

    /** Convenience: updateMatrix() then computeRhs(). */
    void update(int n, int m,
                const int* htP, const int* htI, const double* htX,
                const double* wSq, const double* r);

    /**
     * Levenberg-Marquardt damping mode for applyDamping():
     *   IDENTITY  - D = I (classic Levenberg)
     *   MARQUARDT - D = diag(C) (scale-invariant; Marquardt's choice)
     */
    enum class DampingMode {
        IDENTITY = 0,
        MARQUARDT = 1,
    };

    /**
     * Overwrite the diagonals of C with the LM-damped values:
     *   IDENTITY:  C[k,k] = diagUndamped[k] + lambda
     *   MARQUARDT: C[k,k] = diagUndamped[k] * (1 + lambda)
     *
     * Off-diagonal entries are not touched. The undamped diagonals are
     * snapshotted by updateMatrix(), so this is idempotent: calling with a new
     * lambda restarts from the undamped baseline rather than stacking. lambda
     * == 0 with any mode is equivalent to "no damping".
     *
     * Requires updateMatrix() to have been called first and every C[k,k] to
     * be present in the pattern (which holds for any state-estimation Ht where
     * every parameter has at least one measurement equation).
     */
    void applyDamping(double lambda, DampingMode mode);

    int n() const { return n_; }
    int m() const { return m_; }
    int nnzC() const { return static_cast<int>(ci_.size()); }
    int* cp() { return cp_.data(); }
    int* ci() { return ci_.data(); }
    double* cx() { return cx_.data(); }
    double* rhs() { return rhs_.data(); }

    bool patternBuilt() const { return !cp_.empty(); }

private:
    void buildPattern(int n, int m, const int* htP, const int* htI);
    void validatePattern(int n, int m, const int* htP, const int* htI) const;

    int n_ = 0;
    int m_ = 0;
    bool upperOnly_ = false;

    // Cached Ht pattern and values. htX_ is reused by computeRhs() so callers
    // don't need to pass Ht again when only the residual changes.
    std::vector<int> htP_;
    std::vector<int> htI_;
    std::vector<double> htX_;

    // C in CSC, full symmetric storage.
    std::vector<int> cp_;
    std::vector<int> ci_;
    std::vector<double> cx_;

    // For LM damping: positions of the diagonal entries C[k,k] inside cx_,
    // and a snapshot of those values right after updateMatrix() finishes
    // (i.e. before any damping is applied). applyDamping() rewrites the
    // diagonal entries of cx_ from this snapshot, so calling it again with
    // a different lambda restarts from the undamped baseline.
    std::vector<int> diagIdx_;
    std::vector<double> diagUndamped_;

    // For each rank-1 outer-product term Ht[i,k] * Ht[j,k] contributing to C[i,j],
    // we store the source positions in Ht.x and the destination position in Cx.
    struct Contribution {
        int k;    // column of Ht (= row of H, used to fetch weight)
        int pi;   // index in Ht.x for entry (i, k)
        int pj;   // index in Ht.x for entry (j, k)
        int idx;  // index in Cx for entry (i, j)
    };
    std::vector<Contribution> contribs_;

    std::vector<double> rhs_;
};

}  // namespace gaussnewton

}  // namespace powsybl

#endif // GAUSSNEWTON_COMMON_HPP
