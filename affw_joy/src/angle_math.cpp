/*
 * angle_math.c
 *
 *  Created on: 07.06.2013
 *      Author: AndreR
 */

#include "angle_math.h"

#include <math.h>

float AngleNormalize(float a)
{
	while(a > M_PI)
		a -= 2*M_PI;
	while(a < -M_PI)
		a += 2*M_PI;

	return a;
}

void AngleNormalizePtr(float* a)
{
	while(*a > M_PI)
		*a -= 2*M_PI;
	while(*a < -M_PI)
		*a += 2*M_PI;
}
