set(SOURCE_FILES particle.cpp ParticleManager.cpp particle_system.cpp vec.cpp util.c)
add_executable(ParticleSystem ${SOURCE_FILES})
target_link_libraries (ParticleSystem  ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${CUDA_LIBRARIES})
