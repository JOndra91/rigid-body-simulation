cmake_minimum_required(VERSION 2.8)

project("gmu")

set(qu3e_version 0.1)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Add path to FindOpenCL.cmake and FindSDL2.cmake
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/Modules)

option(qu3e_use_gpu "Build qu3e with OpenCL solver" ON)
option(timers_enabled "Enable timers output" ON)
add_definitions(-DWITH_OCL)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

if(timers_enabled)
  add_definitions(-DTIMERS_ENABLED)
endif()

add_subdirectory(qu3e)
add_subdirectory(imgui)
add_subdirectory(freeglut)
add_subdirectory(src)
