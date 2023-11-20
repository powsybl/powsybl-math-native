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

set(suitesparse_version "7.3.1")
set(suitesparse_archive "v${suitesparse_version}.tar.gz")
set(suitesparse_url https://github.com/DrTimothyAldenDavis/SuiteSparse/archive/${suitesparse_archive})
set(suitesparse_url_hash "MD5=735b8856ce333bf5d8773ba20685c8b8")

include(FetchContent)
FetchContent_Declare(SuiteSparse
  URL ${suitesparse_url}
  URL_HASH ${suitesparse_url_has}
  PATCH_COMMAND git apply --ignore-space-change --ignore-whitespace ${CMAKE_CURRENT_SOURCE_DIR}/patches/suitesparse.patch)

FetchContent_Populate(SuiteSparse)
add_subdirectory(${suitesparse_SOURCE_DIR} ${suitesparse_BINARY_DIR} EXCLUDE_FROM_ALL)

set(SOURCES
    src/lu.cpp
    src/jniwrapper.cpp
)
add_library(math SHARED ${SOURCES})

target_include_directories(math
    PUBLIC
    ${JNI_INCLUDE_DIRS}
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
