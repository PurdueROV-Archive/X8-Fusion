#include "Kalman.h"

//dt is measured in MICROSECONDS, if another timestep is used, change this
//u = accelleration
//z = measured position in millimeters
vect2_f Kalman_location(float u, float z, float dt, direction dir)
{
	//old position and velocity values
	static vect2_f oldX_x = {0,0};
	static vect2_f oldX_y = {0,0};
	static vect2_f oldX_z = {0,0};

	//precomputed gains
	static vect2_f K_x = {0.02, 0.1}; 
	static vect2_f K_y = {0.02, 0.1};
	static vect2_f K_z = {0.02, 0.1};

	vect2_f X;
	vect2_f K;
	vect2_f oldX;
	//set up appropriate vectors for the current state
	switch (dir)
	{
		case DIR_X:
			oldX = oldX_x;
			K = K_x;
			break;
		case DIR_Y:
			oldX = oldX_y;
			K = K_y;
			break;
		case DIR_Z:
			oldX = oldX_z;
			K = K_z;
			break;
	}

	//prediction
	X.a = oldX.a + oldX.b + u * dt * dt / 2;
	X.b = oldX.b + u * dt;

	//error = measured - predicted
	float error = z - X.a;

	//actual
	X = add2_f(X, mul2_f(K, error));

	switch (dir)
	{
		case DIR_X:
			oldX_x = X;
			break;
		case DIR_Y:
			oldX_y = X;
			break;
		case DIR_Z:
			oldX_z = X;
			break;
	}

	return X;
}



//u, z is in milli-radians (1000th of 1 radian)
vect2_f Kalman_rotation(float u, float z, float dt, direction dir)
{
		//old position and velocity values
	static vect2_f oldX_x = {0,0};
	static vect2_f oldX_y = {0,0};
	static vect2_f oldX_z = {0,0};

	//precomputed gains
	static vect2_f K_x = {0.02, -0.2}; 
	static vect2_f K_y = {0.02, -0.2};
	static vect2_f K_z = {0.02, -0.2};

	vect2_f X;
	vect2_f K;
	vect2_f oldX;
	//set up appropriate vectors for the current state
	switch (dir)
	{
		case DIR_X:
			oldX = oldX_x;
			K = K_x;
			break;
		case DIR_Y:
			oldX = oldX_y;
			K = K_y;
			break;
		case DIR_Z:
			oldX = oldX_z;
			K = K_z;
			break;
	}

	//prediction
	X.a = oldX.a + (u - oldX.b)*dt;
	//dont update X.b because we have no data for the x_dot prediction

	//error = measured - predicted
	float error = z - X.a;

	//actual
	X = add2_f(X, mul2_f(K, error));

	switch (dir)
	{
		case DIR_X:
			oldX_x = X;
			break;
		case DIR_Y:
			oldX_y = X;
			break;
		case DIR_Z:
			oldX_z = X;
			break;
	}

	return X;
}


