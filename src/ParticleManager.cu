/*
 * ParticleManager.cu
 *
 *  Created on: Mar 10, 2015
 *      Author: parallels
 */

#include "ParticleManager.h"
#include <cstdlib>
#include <ctime>

extern "C" {
#include "util.h"
}

#include <GL/glew.h>
#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
	#include <GL/freeglut.h>
#endif

#define BLOCK_SIZE 256
#define GRID_SIZE 4

// __global__ void moveParticles(Particle *particles, int size) {
//  int t_x = threadIdx.x;
// 	int b_x = blockIdx.x;	
// 	int in_x = b_x * BLOCK_SIZE + t_x;
// 	
// 	if (in_x < size) {
// 		particles[in_x].move();
// 	}
// }

ParticleManager::ParticleManager(int numParticles, Vec3 boxDimensions) {
	srand(time(NULL));

	float maxX = boxDimensions.x / 2;
	float minX = -maxX;
	float maxY = boxDimensions.y / 2;
	float minY = -maxY;
	float maxZ = boxDimensions.z / 2;
	float minZ = -maxZ;

	float maxVel = boxDimensions.x / 40;

	for (int i = 0; i < numParticles; i++) {
		float xPos = randomFloat(minX, maxX);
		float yPos = randomFloat(minY, maxY);
		float zPos = randomFloat(minZ, maxZ);

		float xVel = randomFloat(0, maxVel);
		float yVel = randomFloat(0, maxVel);
		float zVel = randomFloat(0, maxVel);

		Vec3 pos(xPos, yPos, zPos);
		Vec3 vel(xVel, yVel, zVel);

		mParticles.push_back(Particle(pos, vel, 0.07));
	}
	
	// Initialise device memory for particles
	cudaMalloc((void**) &d_particles, sizeof(Particle) * numParticles);
}

void ParticleManager::update()
{
	// copy host memory to device
// 	cudaMemcpy(d_particles, &mParticles[0], sizeof(Particle) * mParticles.size(), cudaMemcpyHostToDevice);
// 	
// 	moveParticles<<<GRID_SIZE, BLOCK_SIZE>>>(d_particles, mParticles.size());
// 	
// 	// copy result from device to host
// 	cudaMemcpy(&mParticles[0], d_particles, sizeof(Particle) * mParticles.size(), cudaMemcpyDeviceToHost);
}

void ParticleManager::render()
{
    // Draw particles
        for (int i=0; i<mParticles.size(); i++) {	
		glPushMatrix();
		glTranslatef(mParticles[i].position.x, mParticles[i].position.y, mParticles[i].position.z);
		glColor3f(1, 0, 0);
		glutSolidSphere(mParticles[i].radius, 10, 10);
		glPopMatrix();
	}
}
