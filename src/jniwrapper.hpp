/**
 * Copyright (c) 2017, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * @file jniwrapper.cpp
 * @author Geoffroy Jamgotchian <geoffroy.jamgotchian at rte-france.com>
 */

#ifndef JNIWRAPPER_HPP
#define JNIWRAPPER_HPP

#include <string>
#include <jni.h>

namespace powsybl {

namespace jni {

template<typename T>
class JniWrapper {
public:
    JniWrapper(const JniWrapper&) = delete;

    virtual ~JniWrapper() = default;

    JniWrapper& operator=(const JniWrapper&) = delete;

    JNIEnv* env() const {
        return _env;
    }

    const T& obj() const {
        return _obj;
    }

protected:
    JniWrapper(JNIEnv* env, T obj) :
        _env(env),
        _obj(obj) {
    }

    JNIEnv* _env;
    T _obj;
};

class StringUTF : public JniWrapper<jstring> {
public:
    StringUTF(JNIEnv* env, jstring jstr) :
        JniWrapper<jstring>(env, jstr),
        _ptr(nullptr) {
    }

    ~StringUTF() override {
        if (_ptr) {
            _env->ReleaseStringUTFChars(_obj, _ptr);
        }
    }

    size_t length() const {
        return _env->GetStringUTFLength(_obj);
    }

    const char* get() const {
        if (!_ptr) {
            _ptr = _env->GetStringUTFChars(_obj, nullptr);
        }
        return _ptr;
    }

    std::string toStr() const {
        if (!_ptr) {
            _ptr = _env->GetStringUTFChars(_obj, nullptr);
        }
        return {_ptr};
    }

private:
    mutable const char* _ptr;
};

class IntArray : public JniWrapper<jintArray> {
public:
    IntArray(JNIEnv* env, jintArray obj) :
        JniWrapper<jintArray>(env, obj),
        _ptr(nullptr) {
    }

    IntArray(JNIEnv* env, int* ptr, int length) :
        JniWrapper<jintArray>(env, env->NewIntArray(length)),
        _ptr(nullptr) {
        _env->SetIntArrayRegion(_obj, 0, length, (const jint*) ptr);
    }

    ~IntArray() override {
        if (_ptr) {
            _env->ReleaseIntArrayElements(_obj, _ptr, 0);
        }
    }

    size_t length() const {
        return _env->GetArrayLength(_obj);
    }

    int* get() const {
        if (!_ptr) {
            _ptr = _env->GetIntArrayElements(_obj, nullptr);
        }
        return (int*) _ptr;
    }

private:
    mutable jint* _ptr;
};

class DoubleArray : public JniWrapper<jdoubleArray> {
public:
    DoubleArray(JNIEnv* env, jdoubleArray obj) :
        JniWrapper<jdoubleArray>(env, obj),
        _ptr(nullptr) {
    }

    DoubleArray(JNIEnv* env, int length) :
        DoubleArray(env, nullptr, length) {
    }

    DoubleArray(JNIEnv* env, double* ptr, int length) :
        JniWrapper<jdoubleArray>(env, env->NewDoubleArray(length)),
        _ptr(nullptr) {
        if (ptr) {
            _env->SetDoubleArrayRegion(_obj, 0, length, ptr);
        }
    }

    ~DoubleArray() override {
        if (_ptr) {
            _env->ReleaseDoubleArrayElements(_obj, _ptr, 0);
        }
    }

    size_t length() const {
        return _env->GetArrayLength(_obj);
    }

    double* get() const {
        if (!_ptr) {
            _ptr = _env->GetDoubleArrayElements(_obj, nullptr);
        }
        return (double*) _ptr;
    }

private:
    mutable jdouble* _ptr;
};

void throwMathException(JNIEnv* env, const char* msg);
void throwMatrixException(JNIEnv* env, const char* msg);
void throwSolverException(JNIEnv* env, const char* msg);

}  // namespace jni

}  // namespace powsybl

#endif // JNIWRAPPER_HPP
