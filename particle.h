#ifndef __particle_h_
#define __particle_h_

#include "vec.h"

class Particle
{
public:
	Vec3 position;
	Vec3 velocity;
	
	float radius;
	
	Particle(Vec3 pos, Vec3 vel);
};

#endif
