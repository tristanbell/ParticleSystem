#include "particles.cuh"
#include "vec.h"

#include <cuda.h>
#include <stdio.h>
#include <math.h>

static int gridSize;
static int blockSize;
static Particle *d_particles;

__device__ void vectorAdd(Vec3 *v1, Vec3 *v2, Vec3 *out) {
	out->x = v1->x + v2->x;
	out->y = v1->y + v2->y;
	out->z = v1->z + v2->z;
}

__device__ void vectorMinus(Vec3 *v1, Vec3 *v2, Vec3 *out) {
	out->x = v1->x - v2->x;
	out->y = v1->y - v2->y;
	out->z = v1->z - v2->z;
}

__device__ void vectorMul(Vec3 *v1, Vec3 *v2, Vec3 *out) {
	out->x = v1->x * v2->x;
	out->y = v1->y * v2->y;
	out->z = v1->z * v2->z;
}

__device__ void scalarMul(Vec3 *v, float s, Vec3 *out) {
	out->x = v->x * s;
	out->y = v->y * s;
	out->z = v->z * s;
}

__device__ float vectorDot(Vec3 *v1, Vec3 *v2) {
	return v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}

__device__ float vectorLengthSquared(Vec3 *v) {
	return v->x * v->x + v->y * v->y + v->z * v->z;
}

__device__ void vectorReflection(Vec3 *v, Vec3 *normal, Vec3 *out) {
	float dot;
	Vec3 scalVec = *out;

	dot = vectorDot(v, normal);
	scalarMul(normal, 2 * dot, &scalVec);

	vectorMinus(v, &scalVec, out);
}

//__device__ bool particlesCollide(Particle *p1, Particle *p2) {
//	Vec3 collideVec = p1->position;
//	vectorMinus(&(p2->position), &(p1->position), &collideVec);
//
//	float radiuses = p1->radius + p2->radius;
//	float collideDistSq = radiuses * radiuses;
//
//	return vectorLengthSquared(*collideVec) <= collideDistSq;
//}

__global__ void moveParticles(Particle *particles, int size) {
	int t_x = threadIdx.x;
	int b_x = blockIdx.x;
	int in_x = b_x * blockDim.x + t_x;

	if (in_x < size) {
		Vec3 newPos = particles[in_x].position;
		Vec3 vel = particles[in_x].velocity;

		vectorAdd(&newPos, &vel, &newPos);

		// Declare normal for wall collisions
		Vec3 normal = particles[in_x].position;
		bool shouldReflect = false;

		if (newPos.x > 1 || newPos.x < -1) {
			shouldReflect = true;
			normal.x = 1;
			normal.y = 0;
			normal.z = 0;
		}
		if (newPos.y > 1 || newPos.y < -1) {
			shouldReflect = true;
			normal.x = 0;
			normal.y = 1;
			normal.z = 0;
		}
		if (newPos.z > 1 || newPos.z < -1) {
			shouldReflect = true;
			normal.x = 0;
			normal.y = 0;
			normal.z = 1;
		}

		if (shouldReflect) {
			vectorReflection(&vel, &normal, &vel);
		}

		vectorAdd(&(particles[in_x].position), &vel, &newPos);

		particles[in_x].position = newPos;
		particles[in_x].velocity = vel;
	}
}

// The following 2 functions taken from the cuda samples
int iDivUp(int a, int b) {
	return (a % b != 0) ? (a / b + 1) : (a / b);
}

void computeGridSize(int n, int blockSize, int &numBlocks, int &numThreads) {
	numThreads = min(blockSize, n);
	numBlocks = iDivUp(n, numThreads);
}

//__global__ void findCollisions(Particle *particles, int size,
//		CollisionList *outCollisions) {
//	int t_x = threadIdx.x;
//	int b_x = blockIdx.x;
//	int in_x = b_x * BLOCK_SIZE + t_x;
//
//	if (in_x < size) {
//
//	}
//}

void cuda_init(int numParticles) {
	// Initialise device memory for particles
	cudaMalloc((void**) &d_particles, sizeof(Particle) * numParticles);

	computeGridSize(numParticles, 256, gridSize, blockSize);
}

void particles_update(Particle *particles, int particlesSize) {
	// copy host memory to device
	cudaMemcpy(d_particles, particles, sizeof(Particle) * particlesSize,
			cudaMemcpyHostToDevice);

	moveParticles<<<gridSize, blockSize>>>(d_particles, particlesSize);

	// copy result from device to host
	cudaMemcpy(particles, d_particles, sizeof(Particle) * particlesSize,
			cudaMemcpyDeviceToHost);
}
