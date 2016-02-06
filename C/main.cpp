#include "main.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>



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

	vect2_f Rx[5000], Ry[5000];
	for (i = 0; i < 5000; ++i)
	{
		float eRx, eRy;
		eRx = atan(Ayf[i]/Azf[i]);
		eRy = atan(Axf[i]/Azf[i]);

		Rx[i] = Kalman_rotation(Gxf[i], eRx, dtf[i], DIR_X);
		Ry[i] = Kalman_rotation(Gyf[i], eRy, dtf[i], DIR_Y);
	}

	FILE *fpout = fopen("output.txt","w");
	for (i = 0; i < 5000; ++i)
	{
		fprintf(fpout, "%f, %f\t %f, %f\n", Rx[i].a, Rx[i].b, Ry[i].a, Ry[i].b);
	}


	return 0;
}