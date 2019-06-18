# Copyright (c) 2017, RTE (http://www.rte-france.com)
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 2.6)

project(math)

include(ExternalProject)

find_package(JNI REQUIRED)

set(suitesparse_version "5.4.0")
set(suitesparse_url http://faculty.cse.tamu.edu/davis/SuiteSparse/SuiteSparse-${suitesparse_version}.tar.gz)

set(THIRD_PARTY_DIR ${CMAKE_CURRENT_BINARY_DIR}/thirdparty)

# SuiteSparse
ExternalProject_Add(suitesparse
    SOURCE_DIR ${THIRD_PARTY_DIR}/suitesparse_source
    GIT_REPOSITORY "https://github.com/jlblancoc/suitesparse-metis-for-windows.git"
    GIT_TAG "master"
    PATCH_COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/suitesparse_patch.py ${suitesparse_version}
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${THIRD_PARTY_DIR}/suitesparse
               -DBUILD_METIS_DEFAULT=OFF
)

set(SUITE_SPARSE_INCLUDE ${THIRD_PARTY_DIR}/suitesparse/include)
set(SUITE_SPARSE_KLU_LIB ${THIRD_PARTY_DIR}/suitesparse/lib64/${CMAKE_STATIC_LIBRARY_PREFIX}klu${CMAKE_STATIC_LIBRARY_SUFFIX})
set(SUITE_SPARSE_CXSPARSE_LIB ${THIRD_PARTY_DIR}/suitesparse/lib64/${CMAKE_STATIC_LIBRARY_PREFIX}cxsparse${CMAKE_STATIC_LIBRARY_SUFFIX})

set(SOURCES
    src/lu.cpp
    src/jniwrapper.cpp
)
add_library(math SHARED ${SOURCES})

add_dependencies(math suitesparse)

target_include_directories(math
    PUBLIC
    ${JNI_INCLUDE_DIRS}
    ${SUITE_SPARSE_INCLUDE}
)

target_link_libraries(math
    ${SUITE_SPARSE_KLU_LIB}
    ${SUITE_SPARSE_CXSPARSE_LIB}
)
