# Copyright (c) 2017, RTE (http://www.rte-france.com)
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 3.18.0...3.27.7)

project(math)

include(GNUInstallDirs)

find_package(Java REQUIRED)
find_package(JNI REQUIRED)

# compiler options
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(NOPENMP true)

set(THIRD_PARTY_DIR ${CMAKE_CURRENT_BINARY_DIR}/thirdparty)

set(suitesparse_version "7.3.1")
set(suitesparse_archive "v${suitesparse_version}.tar.gz")
set(suitesparse_url https://github.com/DrTimothyAldenDavis/SuiteSparse/archive/${suitesparse_archive})

# SuiteSparse
file(DOWNLOAD ${suitesparse_url} ${THIRD_PARTY_DIR}/${suitesparse_archive})
file(ARCHIVE_EXTRACT INPUT ${THIRD_PARTY_DIR}/${suitesparse_archive} DESTINATION ${THIRD_PARTY_DIR})
set(suitesparse_source ${THIRD_PARTY_DIR}/SuiteSparse-${suitesparse_version})

# patch CMakeLists.txt of SuiteSparse_config module to avoid blas check as it is hard to compile it on windows
# and that we don't need it for the suitesparse modules we beed (amd, colamd, btf, klu and cxsparse)
file(READ ${suitesparse_source}/SuiteSparse_config/CMakeLists.txt FILE_CONTENTS)
string(REPLACE "include ( SuiteSparseBLAS )" "set ( SuiteSparse_BLAS_integer \"int32_t\" )\nset ( BLAS_FOUND true )" FILE_CONTENTS ${FILE_CONTENTS})
file(WRITE ${suitesparse_source}/SuiteSparse_config/CMakeLists.txt ${FILE_CONTENTS})

list(APPEND CMAKE_MODULE_PATH ${suitesparse_source}/SuiteSparse_config/cmake_modules)

add_subdirectory(${suitesparse_source}/SuiteSparse_config/)
add_library(SuiteSparse::SuiteSparseConfig ALIAS SuiteSparseConfig)
add_library(SuiteSparse::SuiteSparseConfig_static ALIAS SuiteSparseConfig_static)

add_subdirectory(${suitesparse_source}/AMD)
add_library(SuiteSparse::AMD ALIAS AMD)
add_library(SuiteSparse::AMD_static ALIAS AMD_static)

add_subdirectory(${suitesparse_source}/COLAMD)
add_library(SuiteSparse::COLAMD ALIAS COLAMD)
add_library(SuiteSparse::COLAMD_static ALIAS COLAMD_static)

add_subdirectory(${suitesparse_source}/BTF)
add_library(SuiteSparse::BTF ALIAS BTF)
add_library(SuiteSparse::BTF_static ALIAS BTF_static)

add_subdirectory(${suitesparse_source}/KLU)
add_library(SuiteSparse::KLU_static ALIAS KLU_static)

add_subdirectory(${suitesparse_source}/CXSparse)
add_library(SuiteSparse::CXSparse_static ALIAS CXSparse_static)

set(SOURCES
    src/lu.cpp
    src/jniwrapper.cpp
)
add_library(math SHARED ${SOURCES})

add_dependencies(math KLU_static CXSparse_static)

target_include_directories(math
    PUBLIC
    ${JNI_INCLUDE_DIRS}
    ${SUITE_SPARSE_INCLUDE}
)

if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64" OR ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "AMD64")
    set(OS_BITS 64)
elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "arm64")
    set(OS_BITS arm64)
else()
    set(OS_BITS 32)
endif()

if(${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
    set(ARCHITECTURE windows_${OS_BITS})
elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    set(ARCHITECTURE linux_${OS_BITS})
elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
    set(ARCHITECTURE osx_${OS_BITS})
else()
    message(FATAL_ERROR "System not supported: ${CMAKE_SYSTEM_NAME}")
endif()

set(MATH_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/target/classes/natives/${ARCHITECTURE})
set_target_properties(math PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${MATH_OUTPUT_DIRECTORY})
set_target_properties(math PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${MATH_OUTPUT_DIRECTORY})
set_target_properties(math PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${MATH_OUTPUT_DIRECTORY})

target_link_libraries(math
        PUBLIC
        SuiteSparse::KLU_static
        SuiteSparse::CXSparse_static
)
