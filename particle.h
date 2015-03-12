#ifndef __particle_h_
#define __particle_h_

#include "vec.h"

struct AABB {
	// Center coordinates
	Vec3 position;
	// Width, height, depth
	Vec3 size;
	
	AABB(Vec3 pos, Vec3 siz) : position(pos), size(siz)
	{ }
};

class Particle
{
public:
	Vec3 position;
	Vec3 velocity;
	
	float radius;
	
	Particle(Vec3 pos, Vec3 vel, float radius);
	
	AABB getAABB();

	void move();

	bool collidesWith(Particle *other);
};

#endif
