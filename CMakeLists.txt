cmake_minimum_required(VERSION 2.8.4)
project(ParticleSystem)

set(GLEW_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include/GL")
set(GLEW_LIBRARY "${CMAKE_SOURCE_DIR}/lib/libGLEW.a")

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
#find_package(CUDA 4.0 QUIET REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/include/GL)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3 -gencode arch=compute_20,code=sm_20)


add_subdirectory(src)

