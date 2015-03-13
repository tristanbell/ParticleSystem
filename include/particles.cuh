#ifndef __particles_h_
#define __particles_h_

#include "particle.h"

void cuda_init(int numParticles);
void particles_update(Particle *particles, int particlesSize);

typedef struct CollisionList {
	int collisionIndex;
	CollisionList *next;
} CollisionList;

//void collisionListAdd(CollisionList *list, int idx) {
//	CollisionList *tmp = list;
//
//	while (tmp->next) {
//		tmp = tmp->next;
//	}
//
//	tmp->next = (CollisionList*)malloc (sizeof(CollisionList));
//	tmp->next->collisionIndex = idx;
//}

#endif
