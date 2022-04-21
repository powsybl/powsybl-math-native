# Copyright (c) 2017, RTE (http://www.rte-france.com)
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.

cmake_minimum_required(VERSION 3.0.2)

project(math)

include(ExternalProject)
include(GNUInstallDirs)

find_package(Java REQUIRED)
find_package(JNI REQUIRED)

set(suitesparse_version "5.8.1")
set(suitesparse_archive "v${suitesparse_version}.tar.gz")
set(suitesparse_url https://github.com/DrTimothyAldenDavis/SuiteSparse/archive/${suitesparse_archive})

message(STATUS "TOTO")
message(STATUS ${CMAKE_CURRENT_BINARY_DIR})

set(THIRD_PARTY_DIR ${CMAKE_CURRENT_BINARY_DIR}/thirdparty)

if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64" OR ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "AMD64")
    set(OS_BITS 64)
elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "arm64")
    set(OS_BITS arm64)
else()
    set(OS_BITS 32)
endif()

# SuiteSparse
if(WIN32)
    set(LAPACK_DIR ${THIRD_PARTY_DIR}/suitesparse_source/lapack_windows/x${OS_BITS})
endif()

file(DOWNLOAD ${suitesparse_url} ${CMAKE_CURRENT_BINARY_DIR}/${suitesparse_archive} SHOW_PROGRESS)

ExternalProject_Add(suitesparse
    SOURCE_DIR ${THIRD_PARTY_DIR}/suitesparse_source
    GIT_REPOSITORY "https://github.com/jlblancoc/suitesparse-metis-for-windows.git"
    GIT_TAG "v1.5.0"
    PATCH_COMMAND ${CMAKE_COMMAND} -E tar xzf ${CMAKE_CURRENT_BINARY_DIR}/${suitesparse_archive}
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${THIRD_PARTY_DIR}/suitesparse
               -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
               -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
               -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
               -DLAPACK_DIR=${LAPACK_DIR}
               -DBUILD_METIS_DEFAULT=OFF
)

set(SUITE_SPARSE_LIB_DIR ${THIRD_PARTY_DIR}/suitesparse/${CMAKE_INSTALL_LIBDIR})
set(SUITE_SPARSE_LIB_PREFIX lib)
set(SUITE_SPARSE_LIB_POSTFIX ${CMAKE_STATIC_LIBRARY_SUFFIX})
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    set(SUITE_SPARSE_LIB_POSTFIX d${CMAKE_STATIC_LIBRARY_SUFFIX})
endif()

if(WIN32)
    set(SUITE_SPARSE_CONFIG_LIB ${SUITE_SPARSE_LIB_DIR}/suitesparseconfig${SUITE_SPARSE_LIB_POSTFIX})
elseif(UNIX)
    set(SUITE_SPARSE_LIB_SUFFIX ${CMAKE_STATIC_LIBRARY_SUFFIX})
    set(SUITE_SPARSE_CONFIG_LIB ${SUITE_SPARSE_LIB_DIR}/${SUITE_SPARSE_LIB_PREFIX}suitesparseconfig${SUITE_SPARSE_LIB_POSTFIX})
else()
    message(FATAL_ERROR "System not supported: ${CMAKE_SYSTEM_NAME}")
endif()

set(SUITE_SPARSE_AMD_LIB ${SUITE_SPARSE_LIB_DIR}/${SUITE_SPARSE_LIB_PREFIX}amd${SUITE_SPARSE_LIB_POSTFIX})
set(SUITE_SPARSE_COLAMD_LIB ${SUITE_SPARSE_LIB_DIR}/${SUITE_SPARSE_LIB_PREFIX}colamd${SUITE_SPARSE_LIB_POSTFIX})
set(SUITE_SPARSE_BTF_LIB ${SUITE_SPARSE_LIB_DIR}/${SUITE_SPARSE_LIB_PREFIX}btf${SUITE_SPARSE_LIB_POSTFIX})
set(SUITE_SPARSE_KLU_LIB ${SUITE_SPARSE_LIB_DIR}/${SUITE_SPARSE_LIB_PREFIX}klu${SUITE_SPARSE_LIB_POSTFIX})
set(SUITE_SPARSE_CXSPARSE_LIB ${SUITE_SPARSE_LIB_DIR}/${SUITE_SPARSE_LIB_PREFIX}cxsparse${SUITE_SPARSE_LIB_POSTFIX})
set(SUITE_SPARSE_INCLUDE ${THIRD_PARTY_DIR}/suitesparse/include)

set(CMAKE_CXX_STANDARD 11)

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

if(${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
    set(ARCHITECTURE windows_${OS_BITS})
elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    set(ARCHITECTURE linux_${OS_BITS})
elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
    set(ARCHITECTURE osx_${OS_BITS})
else()
    message(FATAL_ERROR "System not supported: ${CMAKE_SYSTEM_NAME}")
endif()

if(WIN32)
    set_target_properties(math
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/target/classes/natives/${ARCHITECTURE}"
    )
elseif(UNIX)
    set_target_properties(math
        PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/target/classes/natives/${ARCHITECTURE}"
    )
endif()

target_link_libraries(math
    ${SUITE_SPARSE_KLU_LIB}
    ${SUITE_SPARSE_CXSPARSE_LIB}
    ${SUITE_SPARSE_BTF_LIB}
    ${SUITE_SPARSE_COLAMD_LIB}
    ${SUITE_SPARSE_AMD_LIB}
    ${SUITE_SPARSE_CONFIG_LIB}
)
