cmake_minimum_required(VERSION 2.8)

project("gmu")

set(qu3e_version 1.0)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Add path to FindOpenCL.cmake and FindSDL2.cmake
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/Modules)

add_subdirectory(qu3e)
add_subdirectory(imgui)
add_subdirectory(freeglut)
add_subdirectory(src)
