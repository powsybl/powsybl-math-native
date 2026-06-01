/**
 * Copyright (c) 2026, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * @file gaussnewton_common.cpp
 * @author Gautier Bureau <gautier.bureau at rte-france.com>
 */

#include "gaussnewton_common.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace powsybl {

namespace gaussnewton {

void NormalEquations::buildPattern(int n, int m, const int* htP, const int* htI) {
    n_ = n;
    m_ = m;
    const int nnzHt = htP[m];
    htP_.assign(htP, htP + m + 1);
    htI_.assign(htI, htI + nnzHt);

    // Index entries of Ht by row: for each row i in [0, n), list the (k, position-in-Ht.x)
    // of every entry on that row. This is the dual ("transpose") view of Ht and lets us
    // walk "which columns k of Ht touch row j" when assembling column j of C.
    std::vector<int> rowStart(n + 1, 0);
    for (int p = 0; p < nnzHt; ++p) {
        ++rowStart[htI[p] + 1];
    }
    for (int i = 0; i < n; ++i) {
        rowStart[i + 1] += rowStart[i];
    }
    std::vector<int> rowK(nnzHt);
    std::vector<int> rowPos(nnzHt);
    std::vector<int> cursor(rowStart);
    for (int k = 0; k < m; ++k) {
        for (int p = htP[k]; p < htP[k + 1]; ++p) {
            const int i = htI[p];
            const int slot = cursor[i]++;
            rowK[slot] = k;
            rowPos[slot] = p;
        }
    }

    // Assemble C column-by-column. C[:, j] = sum_k Ht[j, k] * Ht[:, k], so its pattern
    // is the union over k (where Ht[j, k] != 0) of column k's row indices.
    // `mark` doubles as: (1) a per-column "seen" flag during pattern discovery, then
    // (2) a row -> Cx index map while emitting contributions for that column.
    std::vector<int> mark(n, -1);
    std::vector<int> rowOrder;
    cp_.assign(n + 1, 0);
    ci_.clear();
    contribs_.clear();

    for (int j = 0; j < n; ++j) {
        cp_[j] = static_cast<int>(ci_.size());
        rowOrder.clear();

        for (int q = rowStart[j]; q < rowStart[j + 1]; ++q) {
            const int k = rowK[q];
            for (int p = htP[k]; p < htP[k + 1]; ++p) {
                const int i = htI[p];
                if (upperOnly_ && i > j) {
                    continue;
                }
                if (mark[i] != j) {
                    mark[i] = j;
                    rowOrder.push_back(i);
                }
            }
        }

        std::sort(rowOrder.begin(), rowOrder.end());
        for (int i : rowOrder) {
            mark[i] = static_cast<int>(ci_.size());
            ci_.push_back(i);
        }

        for (int q = rowStart[j]; q < rowStart[j + 1]; ++q) {
            const int k = rowK[q];
            const int pj = rowPos[q];
            for (int p = htP[k]; p < htP[k + 1]; ++p) {
                const int i = htI[p];
                if (upperOnly_ && i > j) {
                    continue;
                }
                contribs_.push_back({k, p, pj, mark[i]});
            }
        }

        for (int i : rowOrder) {
            mark[i] = -1;
        }
    }
    cp_[n] = static_cast<int>(ci_.size());

    cx_.assign(ci_.size(), 0.0);
    rhs_.assign(n, 0.0);

    // Locate the diagonal entry of each column for LM damping. Every diagonal
    // C[k,k] must exist in the pattern - it equals sum_i wSq[i]*Ht[k,i]^2 and
    // is structurally present whenever parameter k has at least one
    // measurement equation. If a column k of H is structurally empty we throw
    // early rather than silently producing an undamped diagonal later.
    diagIdx_.assign(n, -1);
    for (int j = 0; j < n; ++j) {
        for (int p = cp_[j]; p < cp_[j + 1]; ++p) {
            if (ci_[p] == j) {
                diagIdx_[j] = p;
                break;
            }
        }
        if (diagIdx_[j] < 0) {
            throw std::runtime_error("NormalEquations: column " + std::to_string(j)
                                     + " of Ht is structurally empty - parameter has no"
                                       " measurements, LM damping has no diagonal to modify");
        }
    }
    diagUndamped_.assign(n, 0.0);
}

void NormalEquations::validatePattern(int n, int m, const int* htP, const int* htI) const {
    if (n != n_ || m != m_) {
        throw std::runtime_error("Ht dimensions changed across solves");
    }
    const int nnzHt = htP[m];
    if (nnzHt != static_cast<int>(htI_.size())) {
        throw std::runtime_error("Ht nnz changed across solves");
    }
    if (std::memcmp(htP, htP_.data(), sizeof(int) * (m + 1)) != 0 ||
        std::memcmp(htI, htI_.data(), sizeof(int) * nnzHt) != 0) {
        throw std::runtime_error("Ht sparsity pattern changed across solves");
    }
}

void NormalEquations::updateMatrix(int n, int m,
                                   const int* htP, const int* htI, const double* htX,
                                   const double* wSq) {
    if (!patternBuilt()) {
        buildPattern(n, m, htP, htI);
    } else {
        validatePattern(n, m, htP, htI);
    }

    // Cache htX values so computeRhs() can rebuild the rhs later without
    // requiring the caller to pass Ht again. The pattern is fixed so the
    // length matches htP_[m_].
    const int nnz = htP[m];
    htX_.assign(htX, htX + nnz);

    std::fill(cx_.begin(), cx_.end(), 0.0);
    for (const Contribution& c : contribs_) {
        cx_[c.idx] += wSq[c.k] * htX[c.pi] * htX[c.pj];
    }

    // Snapshot of un-damped C[k,k]. applyDamping() rewrites cx_[diagIdx_[k]]
    // from this baseline, so re-damping with a new lambda doesn't compound.
    for (int k = 0; k < n_; ++k) {
        diagUndamped_[k] = cx_[diagIdx_[k]];
    }
}

void NormalEquations::applyDamping(double lambda, DampingMode mode) {
    if (diagIdx_.empty()) {
        throw std::runtime_error("applyDamping: call updateMatrix() first");
    }
    if (mode == DampingMode::IDENTITY) {
        for (int k = 0; k < n_; ++k) {
            cx_[diagIdx_[k]] = diagUndamped_[k] + lambda;
        }
    } else {
        // MARQUARDT: C[k,k] = diag_undamped[k] * (1 + lambda).
        const double scale = 1.0 + lambda;
        for (int k = 0; k < n_; ++k) {
            cx_[diagIdx_[k]] = diagUndamped_[k] * scale;
        }
    }
}

void NormalEquations::computeRhs(const double* wSq, const double* r) {
    std::fill(rhs_.begin(), rhs_.end(), 0.0);
    for (int k = 0; k < m_; ++k) {
        const double w_r = wSq[k] * r[k];
        for (int p = htP_[k]; p < htP_[k + 1]; ++p) {
            rhs_[htI_[p]] += htX_[p] * w_r;
        }
    }
}

void NormalEquations::update(int n, int m,
                             const int* htP, const int* htI, const double* htX,
                             const double* wSq, const double* r) {
    updateMatrix(n, m, htP, htI, htX, wSq);
    computeRhs(wSq, r);
}

}  // namespace gaussnewton

}  // namespace powsybl
