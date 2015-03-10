#include "particle.h"

Particle::Particle(Vec3 pos, Vec3 vel, float radius) :
	position(pos),
	velocity(vel),
	radius(radius)
{ }

void Particle::move()
{
	position = position + velocity;
}

bool Particle::collidesWith(Particle *other)
{
	// Get differences between x, y, and z of these two points
	Vec3 differences = position - other->position;
	// Square each difference to get the squared distance
	differences = differences * differences;
	// We want squared distance to avoid needing sqrt()
	float distanceSquared = differences.x + differences.y + differences.z;
	// So we also have to square the minimum distance for a collision
	float collideDistSquared = radius * radius + other->radius * other->radius;

	return distanceSquared <= collideDistSquared;
}
