/*
 * ParticleManager.cpp
 *
 *  Created on: Mar 10, 2015
 *      Author: parallels
 */

#include "ParticleManager.h"
#include "particles.cuh"
#include "shaders.h"
#include <cstdlib>
#include <stdio.h>
#include <ctime>

extern "C" {
#include "util.h"
}GLuint createVBO(GLuint size) {
	GLuint vbo;
	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, size, 0, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	return vbo;
}

GLuint compileProgram(const char *vsource, const char *fsource) {
	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

	glShaderSource(vertexShader, 1, &vsource, 0);
	glShaderSource(fragmentShader, 1, &fsource, 0);

	glCompileShader(vertexShader);
	glCompileShader(fragmentShader);

	GLuint program = glCreateProgram();

	glAttachShader(program, vertexShader);
	glAttachShader(program, fragmentShader);

	glLinkProgram(program);

	// check if program linked
	GLint success = 0;
	glGetProgramiv(program, GL_LINK_STATUS, &success);

	if (!success) {
		char temp[256];
		glGetProgramInfoLog(program, 256, 0, temp);
		printf("Failed to link program:\n%s\n", temp);
		glDeleteProgram(program);
		program = 0;
	}

	return program;
}

ParticleManager::ParticleManager(int numParticles, Vec3 boxDimensions) {
	srand (time(NULL));

	float maxX = boxDimensions.x / 2;
	float minX = -maxX;
	float maxY = boxDimensions.y / 2;
	float minY = -maxY;
	float maxZ = boxDimensions.z / 2;
	float minZ = -maxZ;

	float maxVel = boxDimensions.x / 200;

	float radius = 0.02;

	// Create all of the particles with random position and velocity
	for (int i = 0; i < numParticles; i++) {
		float xPos, yPos, zPos;
		float xVel, yVel, zVel;

		// Split particles in x axis (half left, half right)
		if (i < numParticles / 2) {
			xPos = -0.98f;
			yPos = randomFloat(minY, maxY);
			zPos = randomFloat(minZ, maxZ);

			xVel = randomFloat(0.005f, maxVel);
			yVel = 0;//randomFloat(-maxVel, maxVel);
			zVel = 0;//randomFloat(-maxVel, maxVel);
		}
		else {
			xPos = 0.98f;
			yPos = randomFloat(minY, maxY);
			zPos = randomFloat(minZ, maxZ);

			xVel = randomFloat(-maxVel, 0.005f);
			yVel = 0;//randomFloat(-maxVel, maxVel);
			zVel = 0;//randomFloat(-maxVel, maxVel);
		}

		Vec3 pos(xPos, yPos, zPos);
		Vec3 vel(xVel, yVel, zVel);

		Particle newParticle(pos, vel, radius);
		// Add particle to list
		mParticles.push_back(newParticle);
	}

	cuda_init(&mParticles[0], numParticles);

	// Compile shaders
	mProgram = compileProgram(vertexShader, spherePixelShader);

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

// Update method, called every time the particles move
void ParticleManager::update() {
	particles_update(&mParticles[0], mParticles.size());

	float *pArray = particlesArray();

	// Set the vertex VBO data to the particle positions
	glBindBuffer(GL_ARRAY_BUFFER, mVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 4 * mParticles.size(), pArray,
			GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	delete [] pArray;
}

// Method for drawing the particles on screen, using OpenGL
void ParticleManager::render() {
	glEnable(GL_POINT_SPRITE);
	glTexEnvi(GL_POINT_SPRITE, GL_COORD_REPLACE, GL_TRUE);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE_NV);
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);

	glUseProgram(mProgram);
	glUniform1f(glGetUniformLocation(mProgram, "pointScale"),
			mWindowHeight / tanf(mFOV * 0.5f * (float) M_PI / 180.0f));
	glUniform1f(glGetUniformLocation(mProgram, "pointRadius"),
			mParticles[0].radius);

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
	glUseProgram(0);
	glDisable(GL_POINT_SPRITE);
}
