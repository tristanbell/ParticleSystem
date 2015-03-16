/*
 * ParticleManager.h
 *
 *  Created on: Mar 10, 2015
 *      Author: parallels
 */

#ifndef PARTICLEMANAGER_H_
#define PARTICLEMANAGER_H_

#include <vector>
#include "particle.h"
#include "vec.h"

#include <GL/glew.h>
#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
	#include <GL/freeglut.h>
#endif

class ParticleManager {
protected:
	std::vector<Particle> mParticles;
	GLuint mVBO;
	GLuint mColorVBO;

	float *particlesArray();

public:
	ParticleManager(int numParticles, Vec3 boxDimensions);

	void update();
	void render();
};

#endif /* PARTICLEMANAGER_H_ */
