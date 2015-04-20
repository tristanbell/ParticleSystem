# ParticleSystem
CUDA Particle System with OpenGL.

Rendering taken from the CUDA particle system sample code. Adapted the ideas presented by Tero Karras for collisions (http://devblogs.nvidia.com/parallelforall/thinking-parallel-part-i-collision-detection-gpu/).
## Compile & run
To compile:

1. Create a directory for building `mkdir build`
2. `cd build`
3. `cmake ..`
  * on school machines add `-D CUDA_TOOLKIT_ROOT_DIR=/bham/pd/packages/SL6/x86_64/cudatoolkit-4.0.17/cuda`
4. `make`

To run:

5. `./src/ParticleSystem <number of particles>`
  * If you do not supply a number of particles, 1000 will be used as the default.
