//#include "main.h"
#include "matrices.h"
#include "Kalman.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*
typedef enum {
    DIR_X = 0,
    DIR_Y = 1,
    DIR_Z = 2,
}direction;

vect2_f Kalman_rotation(float u, float z, float dt, direction dir);
vect2_f Kalman_location(float u, float z, float dt, direction dir);
 */

int main(int argc, char const *argv[])
{
	FILE *fp = fopen("IMU_log_03.csv","r");
	int Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz, z, dt;
	float Axf[5000], Ayf[5000], Azf[5000], Gxf[5000], Gyf[5000], 
		  Gzf[5000], Mxf[5000], Myf[5000], Mzf[5000], zf[5000], dtf[5000];
	int i = 0;
	//convert to SI units for the filter
	while ((!feof(fp)) && i < 5000)
	{
		fscanf(fp,"%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d",
			   &Ax, &Ay, &Az, &Gx, &Gy, &Gz, &Mx, &My, &Mz, &z, &dt);
		dtf[i] = (float)dt/1000000.0;   // DT is in us
 		Axf[i] = 4*9.81*Ax/32768.0;   // signed 16 bit 2g
 		Ayf[i] = 4*9.81*Ay/32768.0;
 		Azf[i] = 4*9.81*Az/32768.0;
 		Gxf[i] = 17.4533*Gx/32768.0; // signed 16 bit radians/s
 		Gyf[i] = 17.4533*Gy/32768.0;
 		Gzf[i] = 17.4533*Gz/32768.0;
 		zf[i] = z/1000.0;
 		i++;	
	}

	vect2_f Rx[5000], Ry[5000], estZ[5000];
	for (i = 0; i < 5000; ++i)
	{
		float eRx, eRy;
		eRx = atan(Ayf[i]/Azf[i]);
		eRy = atan(Axf[i]/Azf[i]);

		Rx[i] = Kalman_rotation(Gxf[i], eRx, dtf[i], DIR_X);
		Ry[i] = Kalman_rotation(Gyf[i], eRy, dtf[i], DIR_Y);
        
        
        estZ[i] = Kalman_location(Azf[i],zf[i],dtf[i],DIR_Z);
	}

	FILE *fpout = fopen("output.txt","w");
	for (i = 0; i < 5000; ++i)
	{
		fprintf(fpout, "%f, %f\t %f, %f\t %f, %f\n", Rx[i].a, Rx[i].b, Ry[i].a, Ry[i].b, estZ[i].a, estZ[i].b);
	}


	return 0;
}

/*


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

*/
