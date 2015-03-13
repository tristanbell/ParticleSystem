#include "particles.cuh"

#include <cuda.h>
#include <stdio.h>

#define BLOCK_SIZE 256
#define GRID_SIZE 4

static Particle *d_particles;

__device__ void vectorAdd(Vec3 v1, Vec3 v2, Vec3 *out) {
	out->x = v1.x + v2.x;
	out->y = v1.y + v2.y;
	out->z = v1.z + v2.z;
}

__global__ void moveParticles(Particle *particles, int size) {
	int t_x = threadIdx.x;
	int b_x = blockIdx.x;
	int in_x = b_x * BLOCK_SIZE + t_x;

	if (in_x < size) {
		Vec3 newPos = particles[in_x].position;
		vectorAdd(particles[in_x].position, particles[in_x].velocity, &newPos);

		particles[in_x].position = newPos;
	}
}

void cuda_init(int numParticles) {
	// Initialise device memory for particles
	cudaMalloc((void**) &d_particles, sizeof(Particle) * numParticles);
}

void particles_update(Particle *particles, int particlesSize) {
	// copy host memory to device
	cudaMemcpy(d_particles, particles,
			sizeof(Particle) * particlesSize, cudaMemcpyHostToDevice);

	moveParticles<<<GRID_SIZE, BLOCK_SIZE>>>(d_particles, particlesSize);

	// copy result from device to host
	cudaMemcpy(particles, d_particles,
			sizeof(Particle) * particlesSize, cudaMemcpyDeviceToHost);
}
