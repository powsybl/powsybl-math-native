// Copyright (c) 2017, RTE (http://www.rte-france.com)
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

/**
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
    JNIEnv* env() const { return _env; }
    T obj() const { return _obj; }

protected:
    JniWrapper(JNIEnv* env, T obj)
        : _env(env), _obj(obj) {
    }

    virtual ~JniWrapper() {
    }

    JNIEnv* _env;
    T _obj;
private:
    JniWrapper(const JniWrapper&);
    JniWrapper& operator=(const JniWrapper&);
};

class StringUTF : public JniWrapper<jstring> {
public:
    StringUTF(JNIEnv* env, jstring jstr)
        : JniWrapper<jstring>(env, jstr),
          _ptr(_env->GetStringUTFChars(jstr, NULL)) {
    }

    ~StringUTF() {
        _env->ReleaseStringUTFChars(_obj, _ptr);
    }

    size_t length() const {
        return _env->GetStringUTFLength(_obj);
    }

    const char* get() const {
        return _ptr;
    }

    std::string toStr() const {
        return std::string(_ptr);
    }

private:
    const char* _ptr;
};

class IntArray : public JniWrapper<jintArray> {
public:
    IntArray(JNIEnv* env, jintArray obj)
        : JniWrapper<jintArray>(env, obj),
          _ptr(_env->GetIntArrayElements(obj, 0)) {
    }

    IntArray(JNIEnv* env, int* ptr, int length)
        : JniWrapper<jintArray>(env, env->NewIntArray(length)),
          _ptr((jint*) ptr) {
        _env->SetIntArrayRegion(_obj, 0, length, (const jint*) ptr);
    }

    ~IntArray() {
        _env->ReleaseIntArrayElements(_obj, _ptr, 0);
    }

    size_t length() const {
        return _env->GetArrayLength(_obj);
    }

    int* get() const {
        return (int*) _ptr;
    }

private:
    jint* _ptr;
};

class DoubleArray : public JniWrapper<jdoubleArray> {
public:
    DoubleArray(JNIEnv* env, jdoubleArray obj)
        : JniWrapper<jdoubleArray>(env, obj),
          _ptr(_env->GetDoubleArrayElements(obj, 0)) {
    }

    DoubleArray(JNIEnv* env, double* ptr, int length)
        : JniWrapper<jdoubleArray>(env, env->NewDoubleArray(length)),
          _ptr(ptr) {
        _env->SetDoubleArrayRegion(_obj, 0, length, ptr);
    }

    ~DoubleArray() {
        _env->ReleaseDoubleArrayElements(_obj, _ptr, 0);
    }

    size_t length() const {
        return _env->GetArrayLength(_obj);
    }

    double* get() const {
        return (double*) _ptr;
    }

private:
    jdouble* _ptr;
};

void throwJavaLangRuntimeException(JNIEnv* env, const char* msg);

}

}

#endif // JNIWRAPPER_HPP
