/*
 * ParticleManager.cpp
 *
 *  Created on: Mar 10, 2015
 *      Author: parallels
 */

#include "ParticleManager.h"
#include "particles.cuh"
#include <cstdlib>
#include <ctime>

extern "C" {
#include "util.h"
}

GLuint createVBO(GLuint size) {
	GLuint vbo;
	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, size, 0, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	return vbo;
}

ParticleManager::ParticleManager(int numParticles, Vec3 boxDimensions) {
	srand (time(NULL));

	float maxX = boxDimensions.x / 2;
	float minX = -maxX;
	float maxY = boxDimensions.y / 2;
	float minY = -maxY;
	float maxZ = boxDimensions.z / 2;
	float minZ = -maxZ;

	float maxVel = boxDimensions.x / 400;

	for (int i = 0; i < numParticles; i++) {
		float xPos = randomFloat(minX, maxX);
		float yPos = randomFloat(minY, maxY);
		float zPos = randomFloat(minZ, maxZ);

		float xVel = randomFloat(-maxVel, maxVel);
		float yVel = randomFloat(-maxVel, maxVel);
		float zVel = randomFloat(-maxVel, maxVel);

		Vec3 pos(xPos, yPos, zPos);
		Vec3 vel(xVel, yVel, zVel);

		Particle newParticle(pos, vel, 0.02);
		mParticles.push_back(newParticle);
	}

	cuda_init(numParticles);

	// Generate vertex VBO
	mVBO = createVBO(sizeof(float) * 4 * numParticles);

	mColorVBO = createVBO(sizeof(float) * 4 * numParticles);

	// Fill color buffer (taken from cuda samples)
	glBindBuffer(GL_ARRAY_BUFFER, mColorVBO);
	float *data = (float *) glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
	float *ptr = data;

	for (int i = 0; i < numParticles; i++)
	{
		float t = i / (float) numParticles;

		*ptr++ = rand() / (float) RAND_MAX;
		*ptr++ = rand() / (float) RAND_MAX;
		*ptr++ = rand() / (float) RAND_MAX;
		*ptr++ = 1.0f;
	}

	glUnmapBuffer(GL_ARRAY_BUFFER);
}

float *ParticleManager::particlesArray() {
	float *array = new float[mParticles.size() * 4];

	for (int i = 0; i < mParticles.size(); i++) {
		int idx = i * 4;

		array[idx] = mParticles[i].position.x;
		array[idx + 1] = mParticles[i].position.y;
		array[idx + 2] = mParticles[i].position.z;
		array[idx + 3] = 1;
	}

	return array;
}

void ParticleManager::update() {
	particles_update(&mParticles[0], mParticles.size());

	float *pArray = particlesArray();

	// Set the vertex VBO data to the particle positions
	glBindBuffer(GL_ARRAY_BUFFER, mVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 4 * mParticles.size(), pArray,
			GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	delete pArray;
}

void ParticleManager::render() {
	// Set particle rendering size
	glPointSize(1.5f);

	// Bind the vertex VBO
	glBindBuffer(GL_ARRAY_BUFFER, mVBO);
	glVertexPointer(4, GL_FLOAT, 0, 0);
	glEnableClientState(GL_VERTEX_ARRAY);

	// Bind color VBO
	glBindBuffer(GL_ARRAY_BUFFER, mColorVBO);
	glColorPointer(4, GL_FLOAT, 0, 0);
	glEnableClientState(GL_COLOR_ARRAY);

	glDrawArrays(GL_POINTS, 0, mParticles.size());

	// Clean up
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
}
