#include "particles.cuh"
#include "vec.h"

#include <cuda.h>
#include <stdio.h>
#include <math.h>

static int gridSize;
static int blockSize;
static Particle *d_particles;
static Particle *d_particles_out;

// Vector class for the device
class DeviceVec {

public: // Member variables
	float x;
	float y;
	float z;

public: // Methods
	__device__ DeviceVec(float x, float y, float z) : x(x), y(y), z(z)
	{ }

	__device__ DeviceVec(DeviceVec *v) : x(v->x), y(v->y), z(v->z)
	{ }

	__device__ DeviceVec(Vec3 *v) : x(v->x), y(v->y), z(v->z)
	{ }

	__device__ float lengthSquared() {
		float sum = 0;
		sum += x * x;
		sum += y * y;
		sum += z * z;

		return sum;
	}

	__device__ float length() {
		return sqrt(lengthSquared());
	}

	__device__ void toVec3(Vec3 *toFill)
	{
		toFill->x = x;
		toFill->y = y;
		toFill->z = z;
	}

	__device__ float dot(DeviceVec *other)
	{
		return x * other->x + y * other->y + z * other->z;
	}

	__device__ DeviceVec normalised()
	{
		float len = length();

		DeviceVec normalised(x / len, y / len, z / len);
		return normalised;
	}

	__device__ DeviceVec *reflection(DeviceVec *normal)
	{
		DeviceVec tmp(x, y, z);
		tmp = tmp - (*normal * 2 * this->dot(normal));

		return new DeviceVec(tmp);
	}

	// Operators
	__device__ DeviceVec operator+(const DeviceVec& other)
	{
		DeviceVec newVec(x + other.x, y + other.y, z + other.z);
		return newVec;
	}

	__device__ DeviceVec operator-(const DeviceVec& other)
	{
		DeviceVec newVec(x - other.x, y - other.y, z - other.z);
		return newVec;
	}

	__device__ DeviceVec operator-(const float scalar)
	{
		DeviceVec newVec(x - scalar, y - scalar, z - scalar);
		return newVec;
	}

	__device__ DeviceVec operator*(const DeviceVec& other)
	{
		DeviceVec newVec(x * other.x, y * other.y, z * other.z);
		return newVec;
	}

	__device__ DeviceVec operator*(const float scalar)
	{
		DeviceVec newVec(x * scalar, y * scalar, z * scalar);
		return newVec;
	}
};

__device__ void traverse( BVHNode* node, CollisionList& list, const AABB& queryAABB, Particle* particle, int particleIdx, DeviceVec* forces )
{
	// Bounding box overlaps the query => process node.
	if (checkOverlap(getAABB(node), queryAABB))
	{
		// Leaf node => resolve collision.
		if (node->isLeaf())
			if(particlesCollide(particle, node->particle))
				collide(particle, node->particle, &forces[particleIdx]);
		// Internal node => recurse to children.
		else
		{
			BVHNode* childL = node->leftChild;
			BVHNode* childR = node->rightChilde;
			traverse(childL, list, queryAABB, particle);
			traverse(childR, list, queryAABB, particle);
		}
	}
}

__device__ bool particlesCollide(Particle *p1, Particle *p2) {
	DeviceVec collideVec = DeviceVec(&(p2->position)) - DeviceVec(&(p1->position));

	float radiuses = p1->radius + p2->radius;
	float collideDistSq = radiuses * radiuses;

	return collideVec.lengthSquared() <= collideDistSq;
}

__global__ void moveParticles(Particle *particles, Particle *out, int size) {
	int t_x = threadIdx.x;
	int b_x = blockIdx.x;
	int in_x = b_x * blockDim.x + t_x;

	if (in_x < size) {
		Particle thisParticle = particles[in_x];

		DeviceVec newPosD(&thisParticle.position);
		DeviceVec velD(&thisParticle.velocity);
		
		DeviceVec force(0, 0, 0);

		for (int i = 0; i < size; i++) {
			if (i != in_x) { // Don't consider ourselves
				Particle other = particles[i];

				if (particlesCollide(&thisParticle, &other)) {
					DeviceVec posA(&thisParticle.position);
					DeviceVec posB(&other.position);
					DeviceVec velA(&thisParticle.velocity);
					DeviceVec velB(&other.velocity);
				
					DeviceVec relPos = posB - posA;

					float dist = relPos.length();
					float collideDist = thisParticle.radius + other.radius;
					
					DeviceVec norm = relPos.normalised();

					// spring force
					force = force + norm * -0.5*(collideDist - dist);
				}
			}
		}

		velD = velD + force;

		// Calculate our new desired position
		newPosD = newPosD + velD;

		// Declare normal for wall collisions
		DeviceVec normalD(0, 0, 0);

		bool shouldReflect = false;

		// Set the reflection normal to the wall's normal,
		// if we're touching it
		if ((newPosD.x > 1 && velD.x > 0) || (newPosD.x < -1 && velD.x < 0)) {
			shouldReflect = true;
			normalD.x = 1;
			normalD.y = 0;
			normalD.z = 0;
		}
		if ((newPosD.y > 1 && velD.y > 0) || (newPosD.y < -1 && velD.y < 0)) {
			shouldReflect = true;
			normalD.x = 0;
			normalD.y = 1;
			normalD.z = 0;
		}
		if ((newPosD.z > 1 && velD.z > 0) || (newPosD.z < -1 && velD.z < 0)) {
			shouldReflect = true;
			normalD.x = 0;
			normalD.y = 0;
			normalD.z = 1;
		}

		if (shouldReflect) {
			// Reflect with respect to the wall's normal
			velD = velD.reflection(&normalD);
		}

		// Move this particle
		out[in_x] = thisParticle;

		// Calculate the position after movement
		newPosD = DeviceVec(&thisParticle.position) + velD;

		newPosD.toVec3(&out[in_x].position);
		velD.toVec3(&out[in_x].velocity);
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
	cudaMalloc((void**) &d_particles_out, sizeof(Particle) * numParticles);

	computeGridSize(numParticles, 256, gridSize, blockSize);
}

void particles_update(Particle *particles, int particlesSize) {
	// copy host memory to device
	cudaMemcpy(d_particles, particles, sizeof(Particle) * particlesSize,
			cudaMemcpyHostToDevice);

	moveParticles<<<gridSize, blockSize>>>(d_particles, d_particles_out, particlesSize);

	// copy result from device to host
	cudaMemcpy(particles, d_particles_out, sizeof(Particle) * particlesSize,
			cudaMemcpyDeviceToHost);
}
