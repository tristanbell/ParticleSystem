/*
 * util.c
 *
 *  Created on: Mar 10, 2015
 *      Author: parallels
 */

#include "util.h"
#include "stdlib.h"

/**
 * Generate a random floating point between min and max
 */
float randomFloat(float min, float max)
{
	float r = (float)rand() / (float)RAND_MAX;
	return min + (max - min) * r;
}
