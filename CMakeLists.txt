cmake_minimum_required(VERSION 2.8.4)

# Set up project
project(ParticleSystem)
set(EXECUTABLE_NAME "ParticleSystem")

# Locate and load all necessary packages
set(GLEW_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include/GL")
set(GLEW_LIBRARY "${CMAKE_SOURCE_DIR}/lib/libGLEW.a")

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(CUDA 4.0 QUIET REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/include/GL)

# Compilation flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-arch sm_21 -G -lineinfo)

# Recurse into source directory
add_subdirectory(src)

