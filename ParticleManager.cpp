/*
 * ParticleManager.cpp
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
}

void ParticleManager::update()
{
	for (Particle& p : mParticles) {
		p.move();
	}
}

void ParticleManager::render()
{
    // Draw particles
	for (Particle p : mParticles) {
		glPushMatrix();
		glTranslatef(p.position.x, p.position.y, p.position.z);
		glColor3f(1, 0, 0);
		glutSolidSphere(p.radius, 10, 10);
		glPopMatrix();
	}
}
