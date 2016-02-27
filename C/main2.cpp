//
//  main2.cpp
//  
//
//  Created by Jason King on 2/9/16.
//
//

#include "main2.h"

int main(void)
{
    double loc[101],acc[101],dt[101];
    double x, a, t;
    FILE *fp = fopen("easydata.csv","r");
    for (int i = 0; i < 101; ++i) {
        fscanf(fp, "%lf, %lf, %lf",&x, &a, &t);
        loc[i] = x;
        acc[i] = a;
        dt[i] = t;
    }
    
    fclose(fp);
    
    vect2_f est[101];
    for (int j = 0; j < 101; ++j) {
        est[j] = Kalman_locationZ(acc[j], loc[j], dt[j]);
    }
    
    FILE *fpout = fopen("simpleoutput.txt","w");
    for (int k = 0; k < 101; ++k) {
        fprintf(fpout, "%lf\t%lf\n",est[k].a, est[k].b);
    }
    
    fclose(fpout);
    
    return 0;
}
