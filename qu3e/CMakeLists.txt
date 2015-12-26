cmake_minimum_required(VERSION 2.8)

project("qu3e")

set(qu3e_version 1.1)

option(qu3e_build_shared "Build qu3e shared libraries" OFF)
option(qu3e_build_static "Build qu3e static libraries" ON)
option(qu3e_use_gpu "Build qu3e with OpenCL solver" OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

if(qu3e_use_gpu)
  add_definitions(-DWITH_OCL)
  add_subdirectory(oclUtils)
  set(OCL-Utilities_DIR ${CMAKE_BINARY_DIR}/qu3e/oclUtils)
  find_package(OCL-Utilities REQUIRED)
endif()

add_subdirectory(src)
