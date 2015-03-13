#ifndef __particles_h_
#define __particles_h_

#include "particle.h"

extern void cuda_init(int numParticles);
extern void particles_update(Particle *particles, int particlesSize);

#endif
