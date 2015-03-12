include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/include/GL)

set(SOURCE_FILES particle.cpp ParticleManager.cu particle_system.cpp vec.cpp util.c)

link_directories(${CMAKE_SOURCE_DIR}/lib)

CUDA_ADD_EXECUTABLE(ParticleSystem ${SOURCE_FILES})

target_link_libraries (ParticleSystem  ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${CUDA_LIBRARIES})
