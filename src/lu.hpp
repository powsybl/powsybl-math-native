/**
 * Copyright (c) 2023, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * @file native.cpp
 * @author Geoffroy Jamgotchian <geoffroy.jamgotchian at rte-france.com>
 */

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

}  // namespace jni

}  // namespace powsybl
