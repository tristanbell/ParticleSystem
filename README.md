# ParticleSystem
CUDA Particle System with OpenGL.

Adapted ideas presented by Tero Karras' (http://devblogs.nvidia.com/parallelforall/thinking-parallel-part-i-collision-detection-gpu/)
## Compile & run
To compile:

1. Create a directory for building `mkdir build`
2. `cd build`
3. `cmake ..`
  * on school machines add `-D CUDA_TOOLKIT_ROOT_DIR=/bham/pd/packages/SL6/x86_64/cudatoolkit-4.0.17/cuda`
4. `make`

To run:

5. `./src/ParticleSystem <number of particles>`