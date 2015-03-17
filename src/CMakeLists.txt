set(CPP_SOURCE_FILES particle.cpp particle_system.cpp ParticleManager.cpp vec.cu util.c particles.cu shaders.cpp)

CUDA_ADD_EXECUTABLE(${EXECUTABLE_NAME} ${CPP_SOURCE_FILES})
# Link shared libraries and compiled CUDA libraries
TARGET_LINK_LIBRARIES (${EXECUTABLE_NAME} ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${CUDA_LIBRARIES})
