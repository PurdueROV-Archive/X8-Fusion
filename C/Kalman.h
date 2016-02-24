#ifndef __KALMAN__H
#define __KALMAN__H

#include "matrices.h"

const int DIR_X = 0;
const int DIR_Y = 1;
const int DIR_Z = 2;

vect2_f Kalman_location(float u, float z, float dt, int dir);

vect2_f Kalman_rotation(float u, float z, float dt, int dir);


#endif