#ifndef __KALMAN__H
#define __KALMAN__H

#include "matrices.h"

typedef enum {
	DIR_X = 0,
	DIR_Y = 1,
	DIR_Z = 2,
}direction;

vect2_f Kalman_location(float u, float z, float dt, direction dir);

vect2_f Kalman_rotation(float u, float z, float dt, direction dir);


#endif