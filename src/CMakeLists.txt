set(CPP_SOURCE_FILES particle.cpp particle_system.cpp ParticleManager.cpp vec.cpp util.c particles.cu)

# Compile CUDA libraries
#CUDA_ADD_LIBRARY(particles particles.cu)

CUDA_ADD_EXECUTABLE(${EXECUTABLE_NAME} ${CPP_SOURCE_FILES})
# Link shared libraries and compiled CUDA libraries
TARGET_LINK_LIBRARIES (${EXECUTABLE_NAME} ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${CUDA_LIBRARIES})
