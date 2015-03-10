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

using namespace std;

class ParticleManager {
protected:
	vector<Particle> mParticles;

public:
	ParticleManager(int numParticles, Vec3 boxDimensions);

	void update();
	void render();
};

#endif /* PARTICLEMANAGER_H_ */
