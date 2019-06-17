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

# SuiteSparse
ExternalProject_Add(suitesparse_project
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/suitesparse_source
    GIT_REPOSITORY "https://github.com/jlblancoc/suitesparse-metis-for-windows.git"
    GIT_TAG "master"
    PATCH_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/suitesparse_patch.py ${suitesparse_version}
    BUILD_COMMAND make klu cxsparse
    BUILD_IN_SOURCE 1
    INSTALL_COMMAND ""
)
ExternalProject_Get_Property(suitesparse_project source_dir)
set(suitesparse_source_dir ${source_dir})

set(SOURCES
    src/lu.cpp
    src/jniwrapper.cpp
)
add_library(math SHARED ${SOURCES})

target_include_directories(math
    PUBLIC
    ${JNI_INCLUDE_DIRS}
    ${suitesparse_source_dir}/SuiteSparse/SuiteSparse_config
    ${suitesparse_source_dir}/SuiteSparse/AMD/Include
    ${suitesparse_source_dir}/SuiteSparse/COLAMD/Include
    ${suitesparse_source_dir}/SuiteSparse/BTF/Include
    ${suitesparse_source_dir}/SuiteSparse/KLU/Include
    ${suitesparse_source_dir}/SuiteSparse/CXSparse/Include
)

set(KLU_LIBRARY "")
find_library(KLU_LIBRARY
    NAMES klu
    HINTS "${suitesparse_source_dir}/lib/"
)

target_link_libraries(math
    ${KLU_LIBRARY}
)

