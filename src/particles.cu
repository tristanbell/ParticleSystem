#include "particles.cuh"
#include "vec.h"

#include <stdio.h>
#include <math.h>

#include <cuda.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>

class DeviceVec;
class BVHNode;

static int gridSize;
static int blockSize;
static Particle *d_particles;
static Particle *d_particles_out;
// static int *d_particleIdxs;
static unsigned int *mortonCodes;
static unsigned int *d_morton_codes;
static DeviceVec *d_forces;

/**
 *  3D vector class for the device.
 */
class DeviceVec {

public:
	// Member variables
	float x;
	float y;
	float z;

public:
	// Methods
	__host__ __device__ DeviceVec(float x, float y, float z) :
			x(x), y(y), z(z) {
	}

	__host__ __device__ DeviceVec(DeviceVec *v) :
			x(v->x), y(v->y), z(v->z) {
	}

	__host__ __device__ DeviceVec(Vec3 *v) :
			x(v->x), y(v->y), z(v->z) {
	}

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

	// Accessor methods
	__host__ __device__ float getLeft(float halfWidth) {
		return centre.x - halfWidth;
	}

	__host__ __device__ float getRight(float halfWidth) {
		return centre.x + halfWidth;
	}

	__host__ __device__ float getTop(float halfHeight) {
		return centre.y + halfHeight;
	}

	__host__ __device__ float getBottom(float halfHeight) {
		return centre.y - halfHeight;
	}

	__host__ __device__ float getFront(float halfDepth) {
		return centre.z + halfDepth;
	}

	__host__ __device__ float getBack(float halfDepth) {
		return centre.z - halfDepth;
	}

public:
	__host__ __device__ AABB() :
			centre(0,0,0), width(0), height(0), depth(0) {
	}

	__host__ __device__ AABB(DeviceVec centre, float width, float height, float depth) :
			centre(centre), width(width), height(height), depth(depth) {
	}

	__host__ __device__  static AABB fromParticle(Particle *p) {
		DeviceVec centre(&p->position);
		float diameter = p->radius * 2; // This is width, height, and depth

		return AABB(centre, diameter, diameter, diameter);
	}

	/**
 	 * Function for checking whether this AABB and another intersect
 	 */
	__device__ bool intersects(AABB *other) {
		float halfWidth = width / 2;
		float oHalfWidth = other->width / 2;
		float halfHeight = height / 2;
		float oHalfHeight = other->height / 2;
		float halfDepth = depth / 2;
		float oHalfDepth = other->depth / 2;

		if (getRight(halfWidth) < other->getLeft(oHalfWidth))
			return false;
		if (getLeft(halfWidth) > other->getRight(oHalfWidth))
			return false;
		if (getBottom(halfHeight) > other->getTop(oHalfHeight))
			return false;
		if (getTop(halfHeight) < other->getBottom(oHalfHeight))
			return false;
		if (getFront(halfDepth) < other->getBack(oHalfDepth))
			return false;
		if (getBack(halfDepth) > other->getFront(oHalfDepth))
			return false;

		return true;
	}

	// Get the AABB that is found by combining two AABBs
	AABB aabbUnion(AABB other) {
		float halfWidth = width / 2;
		float oHalfWidth = other.width / 2;
		float halfHeight = height / 2;
		float oHalfHeight = other.height / 2;
		float halfDepth = depth / 2;
		float oHalfDepth = other.depth / 2;

		// Get the extreme values (leftmost, rightmost, topmost, etc.) from either AABB
		float left = min(getLeft(halfWidth), other.getLeft(oHalfWidth));
		float right = max(getRight(halfWidth), other.getRight(oHalfWidth));
		float top = max(getTop(halfHeight), other.getTop(oHalfHeight));
		float bottom = min(getBottom(halfHeight), other.getBottom(oHalfHeight));
		float front = max(getFront(halfDepth), other.getFront(oHalfDepth));
		float back = min(getBack(halfDepth), other.getBack(oHalfDepth));

		// Calculate new width, height and depth based on above calculation
		float newWidth = right - left;
		float newHeight = top - bottom;
		float newDepth = front - back;

		DeviceVec newCentre(left + newWidth/2, bottom + newHeight/2, back + newDepth/2);

		return AABB(newCentre, newWidth, newHeight, newDepth);
	}
};

/**
 * Represents a node in a Bounding Volume Hierarchy (BVH).
 * The BVH is described by its root node, and is a binary
 * tree of AABBs.
 */
struct BVHNode {
	int particleIdx;
	AABB boundingBox;
	int leftChildIdx, rightChildIdx;

	BVHNode() :
		particleIdx(-1), boundingBox(), leftChildIdx(-1), rightChildIdx(-1)
	{ }

	// Constructor creates an internal (non-leaf) node
	BVHNode(AABB aabb, int l, int r) :
		particleIdx(-1), boundingBox(aabb), leftChildIdx(l), rightChildIdx(r)
	{ }

	// Static function for creating a leaf node which directly represents a particle
	static BVHNode leafNode(Particle *p, int idx) {
		// Find the aabb bounding box for this particle
		// -1 represents there are no child nodes of this element
		BVHNode node(AABB::fromParticle(p), -1, -1);
		// Set the particle index
		node.particleIdx = idx;

		return node;
	}

	// Check whether a node is a leaf
	__device__ bool isLeaf() {
		// A node is leaf if both of it's child indexes are -1
		return leftChildIdx == -1 && rightChildIdx == -1;
	}

	// Functions for checking the presence of individual children
	__device__ bool hasLeftChild() {
		return leftChildIdx != -1;
	}
	__device__ bool hasRightChild() {
		return rightChildIdx != -1;
	}
};

static thrust::host_vector<BVHNode> nodes;
static BVHNode *d_nodes;
static int numBVHNodes;

// Device function for checking whether two given particles collide
__device__ bool particlesCollide(Particle *p1, Particle *p2) {
	// Find the vector between the two particles
	DeviceVec collideVec = DeviceVec(&(p2->position)) - DeviceVec(&(p1->position));
	// Find the combined radius of the two particles
	// 
	float radiuses = p1->radius + p2->radius;
	float collideDistSq = radiuses * radiuses;

	return collideVec.lengthSquared() <= collideDistSq;
}

// Resolve a collision between two colliding particles
__device__ void collide(Particle *p1, Particle *p2, DeviceVec *force) {
	DeviceVec posA(&p1->position);
	DeviceVec posB(&p2->position);
	DeviceVec velA(&p1->velocity);
	DeviceVec velB(&p2->velocity);

	// relative Position and velocity
	DeviceVec relPos = posB - posA;
    DeviceVec relVel = velB - velA;

    // Distance between the two particles
	float dist = relPos.length();
	// Minimum distance for these particles to be colliding
	float collideDist = p1->radius + p2->radius;

	DeviceVec norm = relPos.normalised();

	// New force is accumalated in the force parameter
	// spring force
	*force = *force - norm * 0.5f * (collideDist - dist);
	// damping force
    *force = *force + relVel * 0.02f;
}

/**
 * Recursive BVH Tree traversal
 */ 
__device__ void traverse( BVHNode node, BVHNode *nodes, AABB& queryAABB, Particle *particles, int particleIdx, DeviceVec* forces )
{
	if (node.boundingBox.intersects(&queryAABB))
	{
		Particle* particle = &particles[particleIdx];
		// Base case: Leaf node
		if (node.isLeaf()) {
			if (particleIdx != node.particleIdx && particlesCollide(particle, &particles[node.particleIdx])){
				 // Resolve collision.
				collide(particle, &particles[node.particleIdx], &forces[particleIdx]);
			}
		}
		else
		{
			// Recurse over left and right children as necessary
			if (node.hasLeftChild()) {
				BVHNode childL = nodes[node.leftChildIdx];
				traverse(childL, nodes, queryAABB, particles, particleIdx, forces);
			}
			if (node.hasRightChild()) {
				BVHNode childR = nodes[node.rightChildIdx];
				traverse(childR, nodes, queryAABB, particles, particleIdx, forces);
			}
		}
	}
}

/**
 * More efficient iterative traversal
 */
__device__ void traverseIterative( BVHNode *nodes, BVHNode root, AABB& queryAABB, Particle *particles, int particleIdx, DeviceVec* forces )
{
	Particle* particle = &particles[particleIdx];

	// Stack for keeping track of which nodes to traverse next
    BVHNode* stack[64];
    BVHNode** stackPtr = stack;
    // push to stak
    *stackPtr++ = NULL;

    // Start from the root
    BVHNode* node = &root;
    do
    {
    	BVHNode *childL = node->hasLeftChild() ? &nodes[node->leftChildIdx] : NULL;
    	BVHNode *childR = node->hasRightChild() ? &nodes[node->rightChildIdx] : NULL;

    	// Check whether left and right children intersect with the particle in question
    	// Also make sure we are not the not representing the partice itself
        bool overlapL = childL && childL->boundingBox.intersects(&queryAABB) && particleIdx != childL->particleIdx;
        bool overlapR = childR && childR->boundingBox.intersects(&queryAABB) && particleIdx != childR->particleIdx;

        // If there is an overlap and you are already a leaf then resolve the collision
        if (overlapL && childL->isLeaf())
        	collide(particle, &particles[childL->particleIdx], &forces[particleIdx]);
        if (overlapR && childR->isLeaf())
        	collide(particle, &particles[childR->particleIdx], &forces[particleIdx]);

        // If there is an overlap and you are not a leaf then traverse further
        bool traverseL = (overlapL && !childL->isLeaf());
        bool traverseR = (overlapR && !childR->isLeaf());


        if (!traverseL && !traverseR)
        	// if nothing else to traverse, pop from stack
            node = *--stackPtr;
        else
        {
        	// If left needs traversing, set node to be left child otherwise set to right child
        	// This works because we already know that at least one of the nodes need traversing
            node = (traverseL) ? childL : childR;
            if (traverseL && traverseR)
            	// If they actually both need traversing then push right child onto the stack
                *stackPtr++ = childR;
        }
    }
    while (node != NULL);
}

/** 
 * Kernel for moving the particles
 */
__global__ void moveParticles(int bvhRootIdx, BVHNode *nodes, Particle *particles, Particle *out, int size, DeviceVec *forces) {
	int t_x = threadIdx.x;
	int b_x = blockIdx.x;
	int in_x = b_x * blockDim.x + t_x;

	if (in_x < size) {
		Particle thisParticle = particles[in_x];

		DeviceVec newPosD(&thisParticle.position);
		DeviceVec velD(&thisParticle.velocity);

		// Initialise force for this particle
		forces[in_x] = DeviceVec(0, 0, 0);

		// Find the AABB query for the particle question
		AABB query = AABB::fromParticle(&thisParticle);
		// Traverse the BVH - querying for this particle
		traverseIterative( nodes, nodes[bvhRootIdx], query, particles, in_x, forces);

		__syncthreads();

		velD = velD + forces[in_x];

//		 DeviceVec force(0, 0, 0);
//
//		 for (int i = 0; i < size; i++) {
//		 	if (i != in_x) { // Don't consider ourselves
//		 		Particle other = particles[i];
//
//		 		if (particlesCollide(&thisParticle, &other)) {
//		 			collide(&thisParticle, &other, &force);
//		 		}
//		 	}
//		 }
//
//		 velD = velD + force;

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

		// Calculate the position after movement
		newPosD = DeviceVec(&thisParticle.position) + velD;

		newPosD.toVec3(&thisParticle.position);
		velD.toVec3(&thisParticle.velocity);

		// Move this particle
		out[in_x] = thisParticle;
	}
}

// Morton encoding/decoding functions taken from http://devblogs.nvidia.com/parallelforall/thinking-parallel-part-iii-tree-construction-gpu/
////////////////////////////////////////
// Expands a 10-bit integer into 30 bits
// by inserting 2 zeros after each bit.
__host__ __device__ unsigned int expandBits(unsigned int v) {
	v = (v * 0x00010001u) & 0xFF0000FFu;
	v = (v * 0x00000101u) & 0x0F00F00Fu;
	v = (v * 0x00000011u) & 0xC30C30C3u;
	v = (v * 0x00000005u) & 0x49249249u;
	return v;
}

// Calculates a 30-bit Morton code for the
// given 3D point located within the cube [-1,1].
__host__ __device__ unsigned int morton3D(Particle *p) {
	DeviceVec v(&p->position);

	// Shift to scale coordinates between 0 and 1
	float x = (v.x + 1) / 2;
	float y = (v.y + 1) / 2;
	float z = (v.z + 1) / 2;

	x = min(max(x * 1024, 0.0f), 1023.0f);
	y = min(max(y * 1024, 0.0f), 1023.0f);
	z = min(max(z * 1024, 0.0f), 1023.0f);
	unsigned int xx = expandBits((unsigned int) x);
	unsigned int yy = expandBits((unsigned int) y);
	unsigned int zz = expandBits((unsigned int) z);
	return (xx * 4) + (yy * 2) + zz;
}

/**
 * GPU Kernel for creating an array of Morton codes
 */
__global__ void copyMortonCodes(Particle *particles, unsigned int *mortonCodes, int size) {
	int t_x = threadIdx.x;
	int b_x = blockIdx.x;
	int in_x = b_x * blockDim.x + t_x;

	if (in_x < size) {
		mortonCodes[in_x] = morton3D(&particles[in_x]);
	}
}

/**
 * Find the number of leading zeroes in the bit representation of an unsigned integer 
 */ 
int leadingZeros(unsigned int n) {
	// Find how many bits are being used to represent the int 
	int numBits = (int)sizeof(n) * 8;

	// Create a mask that will pass over the number
	// The mask starts as a 1 followed by a series of 0s (e.g. 10000000)
	// The 1 will then shift along to the right
	unsigned int mask = 1 << (numBits-1);

	int numZeros = 0;

	// performing bitwise AND of the number and the mask will indicate whether the number has a 1 in a given position
	while (((n & mask) == 0) && (mask > 0)) {
		// increment the count for each position where we haven't yet found a 1
		numZeros++;
		// Shift the mask along by 1 bit
		mask >>= 1;
	}

	return numZeros;
}

int splitSearch(unsigned int *sortedMortonCodes, unsigned int currentMSB, unsigned int currentBest, int first, int last) {
	int mid = first + ((last - first) + 1)/2;

	int msb = leadingZeros(sortedMortonCodes[0] ^ sortedMortonCodes[mid]);


	if (first == last) {
		if (msb > currentMSB)
			currentBest = first;

		return currentBest;
	}

	if (msb > currentMSB) {
		return splitSearch(sortedMortonCodes, currentMSB, mid, mid + 1, last);
	}
	else {
		return splitSearch(sortedMortonCodes, currentMSB, currentBest, first, mid - 1);
	}
}

int findSplit(unsigned int *sortedMortonCodes, int first, int last) {
//	// Generate an initial guess for most significant differing bit
//	int msb = leadingZeros(sortedMortonCodes[first] ^ sortedMortonCodes[last]);
//	unsigned int currentBest = first;
//
//	return splitSearch(sortedMortonCodes, msb, currentBest, first, last);

	// Identical Morton codes => split the range in the middle.
	unsigned int firstCode = sortedMortonCodes[first];
	unsigned int lastCode = sortedMortonCodes[last];

	if (firstCode == lastCode)
		return (first + last) >> 1;

	// Calculate the number of highest bits that are the same
	// for all objects, using the count-leading-zeros intrinsic.

	int commonPrefix = leadingZeros(firstCode ^ lastCode);

	// Use binary search to find where the next bit differs.
	// Specifically, we are looking for the highest object that
	// shares more than commonPrefix bits with the first one.

	int split = first; // initial guess
	int step = last - first;

	do
	{
		step = (step + 1) >> 1; // exponential decrease
		int newSplit = split + step; // proposed new position

		if (newSplit < last)
		{
			unsigned int splitCode = sortedMortonCodes[newSplit];
			int splitPrefix = leadingZeros(firstCode ^ splitCode);
			if (splitPrefix > commonPrefix)
				split = newSplit; // accept proposal
		}
	}
	while (step > 1);

	return split;
}

// BVH generation adapted from the above link
BVHNode generateBVH(unsigned int *sortedMortonCodes, Particle *particles, int *sortedParticleIdxs, int first, int last, int &numNodes, thrust::host_vector<BVHNode> &nodes) {
	numNodes++;

	// Base case: create a leaf node
	if (first == last) {
		BVHNode node = BVHNode::leafNode(&particles[sortedParticleIdxs[first]], sortedParticleIdxs[first]);

		nodes.push_back(node);
		return node;
	}

	// Find the point to split Morton codes for subtrees
	int splitIdx = findSplit(sortedMortonCodes, first, last);

	// Recursively generate subtrees for the split ranges
	BVHNode left = generateBVH(sortedMortonCodes, particles, sortedParticleIdxs, first, splitIdx, numNodes, nodes);
	int leftIdx = nodes.size() - 1;
	BVHNode right = generateBVH(sortedMortonCodes, particles, sortedParticleIdxs, splitIdx + 1, last, numNodes, nodes);
	int rightIdx = nodes.size() - 1;

	// Node contains union of left and right bounding boxes
	BVHNode node(left.boundingBox.aabbUnion(right.boundingBox), leftIdx, rightIdx);
	nodes.push_back(node);
	return node;
}

// The following 2 functions taken from the cuda samples
int iDivUp(int a, int b) {
	return (a % b != 0) ? (a / b + 1) : (a / b);
}

void computeGridSize(int n, int blockSize, int &numBlocks, int &numThreads) {
	numThreads = min(blockSize, n);
	numBlocks = iDivUp(n, numThreads);
}

void cuda_init(Particle *particles, int numParticles) {
	mortonCodes = (unsigned int*)malloc(sizeof(unsigned int) * numParticles);

	// Initialise device memory for particles
	cudaMalloc((void**) &d_particles, sizeof(Particle) * numParticles);
	cudaMalloc((void**) &d_particles_out, sizeof(Particle) * numParticles);
	cudaMalloc((void**) &d_forces, sizeof(DeviceVec) * numParticles);
	// cudaMalloc((void**) &d_particleIdxs, sizeof(int) * numParticles);
	cudaMalloc((void**) &d_morton_codes, sizeof(unsigned int) * numParticles);

	// Copy over particles
//	cudaMemcpy(d_particles, particles, sizeof(Particle) * numParticles, cudaMemcpyHostToDevice);

	cudaError_t err = cudaGetLastError();
	if (err != cudaSuccess)
	    printf("Memory Allocation Error: %s\n", cudaGetErrorString(err));

	computeGridSize(numParticles, 256, gridSize, blockSize);
}

static int count = 0;

void particles_update(Particle *particles, int particlesSize) {
	// copy host memory to device
	cudaMemcpy(d_particles, particles, sizeof(Particle) * particlesSize, cudaMemcpyHostToDevice);

	int *particleIdxs = (int*)malloc(sizeof(int) * particlesSize);
	for (int i = 0; i < particlesSize; i++) {
		particleIdxs[i] = i;
	}

//	cudaMemcpy(d_particleIdxs, particleIdxs, sizeof(int) * particlesSize, cudaMemcpyHostToDevice);

	copyMortonCodes<<<gridSize, blockSize>>>(d_particles, d_morton_codes, particlesSize);

//	cudaThreadSynchronize();

	cudaError_t err = cudaMemcpy(mortonCodes, d_morton_codes, sizeof(unsigned int) * particlesSize, cudaMemcpyDeviceToHost);

	// Sort Particles by their Morton codes
	thrust::sort_by_key(mortonCodes, mortonCodes + particlesSize, particleIdxs);

//	for (int i = 0; i < particlesSize; i++) {
//		printf("%u, ", mortonCodes[i]);
//	}
//	printf("\n\n\n");

//	printf("Copied. Generating BVH...\n");

	// Generate the BVH
	numBVHNodes = 0;
	nodes.clear();
	BVHNode rootNode = generateBVH(mortonCodes, particles, particleIdxs, 0, particlesSize - 1, numBVHNodes, nodes);
	int rootIndex = nodes.size() - 1;

	// printf("id_x: %.4f, id_y: %.4f, idz_: %.4f\n", rootNode.boundingBox.centre.x, rootNode.boundingBox.centre.y, rootNode.boundingBox.centre.z);
	// for (int i = 0; i < numBVHNodes; ++i)
	// {
	// 	printf("[%d]\tx: %.4f, y: %.4f, z: %.4f\n", i, nodes[i].boundingBox.centre.x, nodes[i].boundingBox.centre.y, nodes[i].boundingBox.centre.z);
	// 	printf("\twidth: %.4f, height: %.4f, depth: %.4f\n", i, nodes[i].boundingBox.width, nodes[i].boundingBox.height, nodes[i].boundingBox.depth);
	// 	printf("\t\tleftChildIdx: %d, rightChildIdx: %d\n", nodes[i].leftChildIdx, nodes[i].rightChildIdx);
	// 	printf("\t\tisLeaf(): %d\n", nodes[i].isLeaf());
	// }

//	printf("Generated.\n");

	free (particleIdxs);

//	printf("Nodes: %d\n", numBVHNodes);

	err = cudaMalloc((void**) &d_nodes, sizeof(BVHNode) * numBVHNodes);
	if (err != cudaSuccess)
		printf("[%d] Alloc Nodes Error: %s\n", count, cudaGetErrorString(err));

	err = cudaMemcpy(d_nodes, thrust::raw_pointer_cast(&nodes[0]), sizeof(BVHNode) * nodes.size(), cudaMemcpyHostToDevice);
	if (err != cudaSuccess)
		printf("[%d] Copy Nodes Error: %s\n", count, cudaGetErrorString(err));

//	printf("Copied. Moving...\n");

//	cudaEvent_t start, stop;
//	cudaEventCreate(&start);
//	cudaEventCreate(&stop);
//
//	cudaEventRecord(start);
	moveParticles<<<gridSize, blockSize>>>(rootIndex, d_nodes, d_particles, d_particles_out, particlesSize, d_forces);
	cudaMemcpy(particles, d_particles_out, sizeof(Particle) * particlesSize, cudaMemcpyDeviceToHost);
//	cudaEventRecord(stop);
//
//	cudaEventSynchronize(stop);
//	float milliseconds = 0;
//	cudaEventElapsedTime(&milliseconds, start, stop);
//
//	printf("Time: %fms\n", milliseconds);

//	cudaThreadSynchronize();

//	err = cudaGetLastError();
//	if (err != cudaSuccess)
//	    printf("[%d] Move Particles Error: %s\n", count++, cudaGetErrorString(err));

	// copy result from device to host

	cudaFree(d_nodes);
}
