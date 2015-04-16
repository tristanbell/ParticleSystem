#include "particles.cuh"
#include "vec.h"

#include <cuda.h>
#include <stdio.h>
#include <math.h>

static int gridSize;
static int blockSize;
static Particle *d_particles;
static Particle *d_particles_out;

/**
 *  3D vector class for the device.
 */
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

	__device__ void toVec3(Vec3 *toFill) {
		toFill->x = x;
		toFill->y = y;
		toFill->z = z;
	}

	__device__ float dot(DeviceVec *other) {
		return x * other->x + y * other->y + z * other->z;
	}

	__device__ DeviceVec normalised() {
		float len = length();

		DeviceVec normalised(x / len, y / len, z / len);
		return normalised;
	}

	__device__ DeviceVec *reflection(DeviceVec *normal) {
		DeviceVec tmp(x, y, z);
		tmp = tmp - (*normal * 2 * this->dot(normal));

		return new DeviceVec(tmp);
	}

	// Operators
	__device__ DeviceVec operator+(const DeviceVec& other) {
		DeviceVec newVec(x + other.x, y + other.y, z + other.z);
		return newVec;
	}

	__device__ DeviceVec operator-(const DeviceVec& other) {
		DeviceVec newVec(x - other.x, y - other.y, z - other.z);
		return newVec;
	}

	__device__ DeviceVec operator-(const float scalar) {
		DeviceVec newVec(x - scalar, y - scalar, z - scalar);
		return newVec;
	}

	__device__ DeviceVec operator*(const DeviceVec& other) {
		DeviceVec newVec(x * other.x, y * other.y, z * other.z);
		return newVec;
	}

	__device__ DeviceVec operator*(const float scalar) {
		DeviceVec newVec(x * scalar, y * scalar, z * scalar);
		return newVec;
	}
};

/**
 * Axis-Aligned Bounding Box (AABB) class.
 * Used for collision detection within the Bounding Volume
 * Hierarchy structure.
 */
class AABB {
private:
	DeviceVec centre;
	float width;
	float height;
	float depth;

	__device__ float getLeft(float halfWidth)
	{ return centre.x - halfWidth; }

	__device__ float getRight(float halfWidth)
	{ return centre.x + halfWidth; }

	__device__ float getTop(float halfHeight)
	{ return centre.y + halfHeight; }

	__device__ float getBottom(float halfHeight)
	{ return centre.y - halfHeight; }

	__device__ float getFront(float halfDepth)
	{ return centre.z + halfDepth; }

	__device__ float getBack(float halfDepth)
	{ return centre.z - halfDepth; }

public:
	__device__ AABB(DeviceVec centre, float width, float height, float depth) :
		centre(centre), width(width), height(height), depth(depth)
	{ }

	__device__ bool intersects(AABB *other)
	{
		float halfWidth = width / 2;
		float oHalfWidth = other->width / 2;
		float halfHeight = height / 2;
		float oHalfHeight = other->height / 2;
		float halfDepth = depth / 2;
		float oHalfDepth = other->depth / 2;

		if (getRight(halfWidth) <= other->getLeft(oHalfWidth)) return false;
		if (getLeft(halfWidth) >= other->getRight(oHalfWidth)) return false;
		if (getBottom(halfHeight) >= other->getTop(oHalfHeight)) return false;
		if (getTop(halfHeight) <= other->getBottom(oHalfHeight)) return false;
		if (getFront(halfDepth) <= other->getBack(oHalfDepth)) return false;
		if (getBack(halfDepth) >= other->getFront(oHalfDepth)) return false;

		return true;
	}
};

/**
 * Represents a node in a Bounding Volume Hierarchy (BVH).
 * The BVH is described by its root node, and is a binary
 * tree of AABBs.
 */
struct BVHNode {
	Particle *particle;
	AABB *boundingBox;
	BVHNode *leftChild, *rightChild;

	__device__ BVHNode *leafNode(AABB *aabb) {
		boundingBox = aabb;
		leftChild = NULL;
		rightChild = NULL;
	}

	__device__ bool isLeaf()
	{ return !(leftChild || rightChild); }
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

__device__ void collide(Particle *p1, Particle *p2, DeviceVec *force) {
	DeviceVec posA(&p1->position);
	DeviceVec posB(&p2->position);

	DeviceVec relPos = posB - posA;

	float dist = relPos.length();
	float collideDist = thisParticle.radius + other.radius;

	DeviceVec norm = relPos.normalised();

	// spring force
	*force = *force + norm * -0.5*(collideDist - dist);
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
					collide(&thisParticle, &other, &force);
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

// Morton encoding/decoding functions taken from http://devblogs.nvidia.com/parallelforall/thinking-parallel-part-iii-tree-construction-gpu/
////////////////////////////////////////
// Expands a 10-bit integer into 30 bits
// by inserting 2 zeros after each bit.
unsigned int expandBits(unsigned int v)
{
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}

// Calculates a 30-bit Morton code for the
// given 3D point located within the cube [-1,1].
unsigned int morton3D(float x, float y, float z)
{
	// Shift to scale coordinates between 0 and 1
	x = (x + 1) / 2;
	y = (y + 1) / 2;
	z = (z + 1) / 2;

    x = min(max(x << 10, 0.0f), 1023.0f);
    y = min(max(y << 10, 0.0f), 1023.0f);
    z = min(max(z << 10, 0.0f), 1023.0f);
    unsigned int xx = expandBits((unsigned int)x);
    unsigned int yy = expandBits((unsigned int)y);
    unsigned int zz = expandBits((unsigned int)z);
    return xx << 2 + yy << 1 + zz;
}

// The following 2 functions taken from the cuda samples
int iDivUp(int a, int b) {
	return (a % b != 0) ? (a / b + 1) : (a / b);
}

void computeGridSize(int n, int blockSize, int &numBlocks, int &numThreads) {
	numThreads = min(blockSize, n);
	numBlocks = iDivUp(n, numThreads);
}

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
