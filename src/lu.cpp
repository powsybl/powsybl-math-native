/**
 * Copyright (c) 2017, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * @file lu.cpp
 * @author Geoffroy Jamgotchian <geoffroy.jamgotchian at rte-france.com>
 */

#include <string>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <klu.h>
#include <cs.h>
#include "jniwrapper.hpp"

namespace powsybl {

namespace jni {

class ComPowsyblMathMatrixSparseMatrix : public JniWrapper<jobject> {
public:
    ComPowsyblMathMatrixSparseMatrix(JNIEnv* env, int m, int n, const IntArray& ap, const IntArray& ai, const DoubleArray& ax);

    static void init(JNIEnv* env);

private:
    static jclass _cls;
    static jmethodID _constructor; 
};

jclass ComPowsyblMathMatrixSparseMatrix::_cls = nullptr;
jmethodID ComPowsyblMathMatrixSparseMatrix::_constructor = nullptr;

void ComPowsyblMathMatrixSparseMatrix::init(JNIEnv* env) {
    jclass localCls = env->FindClass("com/powsybl/math/matrix/SparseMatrix");
    _cls = reinterpret_cast<jclass>(env->NewGlobalRef(localCls));
    _constructor = env->GetMethodID(_cls, "<init>", "(II[I[I[D)V");
}

ComPowsyblMathMatrixSparseMatrix::ComPowsyblMathMatrixSparseMatrix(JNIEnv* env, int m, int n, const IntArray& ap, const IntArray& ai, const DoubleArray& ax) :
    JniWrapper<jobject>(env, env->NewObject(_cls, _constructor, m, n, ap.obj(), ai.obj(), ax.obj())) {
}

}  // namespace jni

}  // namespace powsybl

class LUContext {
public:
    LUContext() = default;

    LUContext(const LUContext&) = delete;

    ~LUContext() = default;

    LUContext& operator=(const LUContext&) = delete;

    std::string error() const;

public:
    klu_symbolic* symbolic;
    klu_numeric* numeric;
    klu_common common;
};

std::string LUContext::error() const {
    switch (common.status) {
        case KLU_OK: return "KLU_OK";
        case KLU_SINGULAR: return "KLU_SINGULAR";
        case KLU_OUT_OF_MEMORY: return "KLU_OUT_OF_MEMORY";
        case KLU_INVALID: return "KLU_INVALID";
        case KLU_TOO_LARGE: return "KLU_TOO_LARGE";
        default: throw std::runtime_error("Unknown KLU status");
    }
}

class LUContextManager {
public:
    LUContextManager() = default;

    LUContextManager(const LUContextManager&) = delete;

    ~LUContextManager() = default;

    LUContextManager& operator=(const LUContextManager&) = delete;

    LUContext& createContext(const std::string& id);

    LUContext& findContext(const std::string& id);

    void removeContext(const std::string& id);

private:
    std::map<std::string, std::unique_ptr<LUContext>> _contexts;
    std::mutex _mutex;
};

LUContext& LUContextManager::createContext(const std::string& id) {
    std::lock_guard<std::mutex> lk(_mutex);
    if (_contexts.find(id) != _contexts.end()) {
        throw std::runtime_error("Context " + id + " already exists");
    }
    std::unique_ptr<LUContext> context(new LUContext());
    auto it = _contexts.insert(std::make_pair(id, std::move(context)));
    return *it.first->second;
}

LUContext& LUContextManager::findContext(const std::string& id) {
    std::lock_guard<std::mutex> lk(_mutex);
    auto it = _contexts.find(id);
    if (it == _contexts.end()) {
        throw std::runtime_error("Context " + id + " not found");
    }
    return *it->second;
}

void LUContextManager::removeContext(const std::string& id) {
    std::lock_guard<std::mutex> lk(_mutex);
    _contexts.erase(id);
}

std::unique_ptr<LUContextManager> MANAGER(new LUContextManager());

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Class:     com_powsybl_math_matrix_SparseLUDecomposition
 * Method:    init
 * Signature: (Ljava/lang/String;[I[I[D)V
 */
JNIEXPORT void JNICALL Java_com_powsybl_math_matrix_SparseLUDecomposition_init(JNIEnv * env, jobject, jstring j_id, jintArray j_ap, jintArray j_ai, jdoubleArray j_ax) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        powsybl::jni::IntArray ap(env, j_ap);
        powsybl::jni::IntArray ai(env, j_ai);
        powsybl::jni::DoubleArray ax(env, j_ax);

        LUContext& context = MANAGER->createContext(id);
 
        if (klu_defaults(&context.common) == 0) {
            throw std::runtime_error("klu_defaults error " + context.error());
        }

        context.symbolic = klu_analyze(ap.length()-1, ap.get(), ai.get(), &context.common);
        if (!context.symbolic) {
            throw std::runtime_error("klu_analyze error " + context.error());
        }
        context.numeric = klu_factor(ap.get(), ai.get(), ax.get(), context.symbolic, &context.common);
        if (!context.numeric) {
            throw std::runtime_error("klu_factor error " + context.error());
        }
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
}

/*
 * Class:     com_powsybl_math_matrix_SparseLUDecomposition
 * Method:    update
 * Signature: (Ljava/lang/String;[I[I[DD)D
 */
JNIEXPORT jdouble JNICALL Java_com_powsybl_math_matrix_SparseLUDecomposition_update(JNIEnv * env, jobject, jstring j_id, jintArray j_ap, jintArray j_ai, jdoubleArray j_ax,
                                                                                    jdouble rgrowthThreshold) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        powsybl::jni::IntArray ap(env, j_ap);
        powsybl::jni::IntArray ai(env, j_ai);
        powsybl::jni::DoubleArray ax(env, j_ax);

        LUContext& context = MANAGER->findContext(id);
        if (rgrowthThreshold > 0) {
            int ok = klu_refactor(ap.get(), ai.get(), ax.get(), context.symbolic, context.numeric, &context.common);
            if (ok == 0) {
                throw std::runtime_error("klu_refactor error " + context.error());
            }
            ok = klu_rgrowth(ap.get(), ai.get(), ax.get(), context.symbolic, context.numeric, &context.common);
            if (ok == 0) {
                throw std::runtime_error("klu_rgrowth error " + context.error());
            }
        }
        // if rgrowth is too small we have to do a whole factorization
        if (rgrowthThreshold <= 0 || context.common.rgrowth < rgrowthThreshold) {
            if (klu_free_numeric(&context.numeric, &context.common) == 0) {
                throw std::runtime_error("klu_free_numeric error " + context.error());
            }
            context.numeric = klu_factor(ap.get(), ai.get(), ax.get(), context.symbolic, &context.common);
            if (!context.numeric) {
                throw std::runtime_error("klu_factor error " + context.error());
            }
        }
        return context.common.rgrowth;
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
}

/*
 * Class:     com_powsybl_math_matrix_SparseLUDecomposition
 * Method:    release
 * Signature: (Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_com_powsybl_math_matrix_SparseLUDecomposition_release(JNIEnv * env, jobject, jstring j_id) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();

        LUContext& context = MANAGER->findContext(id);

        if (klu_free_symbolic(&context.symbolic, &context.common) == 0) {
            throw std::runtime_error("klu_free_symbolic error " + context.error());
        }
        if (klu_free_numeric(&context.numeric, &context.common) == 0) {
            throw std::runtime_error("klu_free_numeric error " + context.error());
        }

        MANAGER->removeContext(id);
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
}

/*
 * Class:     com_powsybl_math_matrix_SparseLUDecomposition
 * Method:    solve
 * Signature: (Ljava/lang/String;[DZ)V
 */
JNIEXPORT void JNICALL Java_com_powsybl_math_matrix_SparseLUDecomposition_solve(JNIEnv * env, jobject, jstring j_id, jdoubleArray j_b, jboolean transpose) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        powsybl::jni::DoubleArray b(env, j_b);

        LUContext& context = MANAGER->findContext(id);

        if (transpose) {
            if (klu_tsolve(context.symbolic, context.numeric, b.length(), 1, b.get(), &context.common) == 0) {
                throw std::runtime_error("klu_tsolve error " + context.error());
            }
        } else {
            if (klu_solve(context.symbolic, context.numeric, b.length(), 1, b.get(), &context.common) == 0) {
                throw std::runtime_error("klu_solve error " + context.error());
            }
        }
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
}

/*
 * Class:     com_powsybl_math_matrix_SparseLUDecomposition
 * Method:    solve2
 * Signature: (Ljava/lang/String;IILjava/nio/ByteBuffer;Z)V
 */
JNIEXPORT void JNICALL Java_com_powsybl_math_matrix_SparseLUDecomposition_solve2(JNIEnv * env, jobject, jstring j_id, jint m, jint n, jobject j_b, jboolean transpose) {
    try {
        std::string id = powsybl::jni::StringUTF(env, j_id).toStr();
        auto* b = static_cast<double*>(env->GetDirectBufferAddress(j_b));
        if (!b) {
           throw std::runtime_error("GetDirectBufferAddress error");
        }

        LUContext& context = MANAGER->findContext(id);

        if (transpose) {
            if (klu_tsolve(context.symbolic, context.numeric, m, n, b, &context.common) == 0) {
                throw std::runtime_error("klu_tsolve error " + context.error());
            }
        } else {
            if (klu_solve(context.symbolic, context.numeric, m, n, b, &context.common) == 0) {
                throw std::runtime_error("klu_solve error " + context.error());
            }
        }
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
}

/*
 * Class:     com_powsybl_math_matrix_SparseMatrix
 * Method:    times
 * Signature: (II[I[I[DII[I[I[D)Lcom/powsybl/math/matrix/SparseMatrix;
 */
JNIEXPORT jobject JNICALL Java_com_powsybl_math_matrix_SparseMatrix_times(JNIEnv * env, jobject, jint m1, jint n1, jintArray j_ap1, jintArray j_ai1, jdoubleArray j_ax1, 
                                                                          jint m2, jint n2, jintArray j_ap2, jintArray j_ai2, jdoubleArray j_ax2) {
    try {
        powsybl::jni::IntArray ap1(env, j_ap1);
        powsybl::jni::IntArray ai1(env, j_ai1);
        powsybl::jni::DoubleArray ax1(env, j_ax1);
        powsybl::jni::IntArray ap2(env, j_ap2);
        powsybl::jni::IntArray ai2(env, j_ai2);
        powsybl::jni::DoubleArray ax2(env, j_ax2);

        cs_di a1;
        a1.nz = -1;
        a1.nzmax = ax1.length();
        a1.m = m1;
        a1.n = n1;
        a1.p = ap1.get();
        a1.i = ai1.get();
        a1.x = ax1.get();

        cs_di a2;
        a2.nz = -1;
        a2.nzmax = ax2.length();
        a2.m = m2;
        a2.n = n2;
        a2.p = ap2.get();
        a2.i = ai2.get();
        a2.x = ax2.get();

        cs_di* a3 = cs_di_multiply(&a1, &a2);

/*
        cs_di_print(&a1, 0);
        cs_di_print(&a2, 0);
        cs_di_print(a3, 0);
*/
    
        powsybl::jni::IntArray ap3(env, a3->p, a3->n + 1);
        powsybl::jni::IntArray ai3(env, a3->i, a3->nzmax);
        powsybl::jni::DoubleArray ax3(env, a3->x, a3->nzmax);
        jobject matrix = powsybl::jni::ComPowsyblMathMatrixSparseMatrix(env, a3->m, a3->n, ap3, ai3, ax3).obj();

        cs_free(a3);

        return matrix;
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
    return nullptr;
}

/*
 * Class:     com_powsybl_math_matrix_SparseMatrix
 * Method:    transpose
 * Signature: (II[I[I[D)Lcom/powsybl/math/matrix/SparseMatrix;
 */
JNIEXPORT jobject JNICALL Java_com_powsybl_math_matrix_SparseMatrix_transpose(JNIEnv * env, jobject, jint m, jint n, jintArray j_ap, jintArray j_ai, jdoubleArray j_ax) {
    try {
        powsybl::jni::IntArray ap(env, j_ap);
        powsybl::jni::IntArray ai(env, j_ai);
        powsybl::jni::DoubleArray ax(env, j_ax);

        cs_di a;
        a.nz = -1;
        a.nzmax = ax.length();
        a.m = m;
        a.n = n;
        a.p = ap.get();
        a.i = ai.get();
        a.x = ax.get();

        cs_di* at = cs_di_transpose(&a, 1);

        powsybl::jni::IntArray apt(env, at->p, at->n + 1);
        powsybl::jni::IntArray ait(env, at->i, at->nzmax);
        powsybl::jni::DoubleArray axt(env, at->x, at->nzmax);
        jobject matrix = powsybl::jni::ComPowsyblMathMatrixSparseMatrix(env, at->m, at->n, apt, ait, axt).obj();

        cs_free(at);

        return matrix;
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
    return nullptr;
}

/*
 * Class:     com_powsybl_math_matrix_SparseMatrix
 * Method:    add
 * Signature: (II[I[I[DII[I[I[DDD)Lcom/powsybl/math/matrix/SparseMatrix;
 */
JNIEXPORT jobject JNICALL Java_com_powsybl_math_matrix_SparseMatrix_add(JNIEnv * env, jobject,
                                                                        jint m1, jint n1, jintArray j_ap1, jintArray j_ai1, jdoubleArray j_ax1,
                                                                        jint m2, jint n2, jintArray j_ap2, jintArray j_ai2, jdoubleArray j_ax2,
                                                                        jdouble alpha, jdouble beta) {
    try {
        powsybl::jni::IntArray ap1(env, j_ap1);
        powsybl::jni::IntArray ai1(env, j_ai1);
        powsybl::jni::DoubleArray ax1(env, j_ax1);
        powsybl::jni::IntArray ap2(env, j_ap2);
        powsybl::jni::IntArray ai2(env, j_ai2);
        powsybl::jni::DoubleArray ax2(env, j_ax2);

        cs_di a1;
        a1.nz = -1;
        a1.nzmax = ax1.length();
        a1.m = m1;
        a1.n = n1;
        a1.p = ap1.get();
        a1.i = ai1.get();
        a1.x = ax1.get();

        cs_di a2;
        a2.nz = -1;
        a2.nzmax = ax2.length();
        a2.m = m2;
        a2.n = n2;
        a2.p = ap2.get();
        a2.i = ai2.get();
        a2.x = ax2.get();

        cs_di* a3 = cs_di_add(&a1, &a2, alpha, beta);

        powsybl::jni::IntArray ap3(env, a3->p, a3->n + 1);
        powsybl::jni::IntArray ai3(env, a3->i, a3->nzmax);
        powsybl::jni::DoubleArray ax3(env, a3->x, a3->nzmax);
        jobject matrix = powsybl::jni::ComPowsyblMathMatrixSparseMatrix(env, a3->m, a3->n, ap3, ai3, ax3).obj();

        cs_free(a3);

        return matrix;
    } catch (const std::exception& e) {
        powsybl::jni::throwMatrixException(env, e.what());
    } catch (...) {
        powsybl::jni::throwMatrixException(env, "Unknown exception");
    }
    return nullptr;
}

#ifdef __cplusplus
}
#endif
