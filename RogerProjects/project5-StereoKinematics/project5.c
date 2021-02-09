/*************************************************************************/
/* File:        project5.c                                               */
/* Description: User project #5 - empty project directory for project    */
/*              developement                                             */
/* Date:        01-2015                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

// void stereo_observation(Robot* roger, Observation* obs);
// Observation obs;

void stereo_observation(roger, obs) 
Robot* roger; 
Observation* obs;
{
  double ur, ul;
  double wTb[4][4], pos_b[4], wRb[2][2], pos_w[4];
  double gammaR, gammaL, lambdaL; 
  double K; 
  double bRw[2][2], mat2X2[2][2];
  double scale = 0.001;
  double jacobi_m[2][2]; 
  double jacobi_transpose[2][2];
  double cov_b[2][2];   
  
  if(average_red_pixel(roger, &ul, &ur)) {
    
    gammaR = roger->eye_theta[RIGHT] + atan2(ur - 63.5, FOCAL_LENGTH);
    gammaL = roger->eye_theta[LEFT] + atan2(ul - 63.5, FOCAL_LENGTH);
    
    if ((gammaR - gammaL) == 0.0) lambdaL = 20.0;  // that's 20 meters! (almost infinity)
    else lambdaL = 2.0*BASELINE*cos(gammaR) / sin(gammaR - gammaL);    

    pos_b[X] = lambdaL * cos(gammaL);
    pos_b[Y] = BASELINE + lambdaL * sin(gammaL);
    pos_b[2] = 0.0;
    pos_b[3] = 1.0; 

    // // convert into world frame
    construct_wTb(roger->base_position, wTb);
    matrix_mult(4, 4, wTb, 1, pos_b, pos_w);

    obs->pos[X] = pos_w[X];
    obs->pos[Y] = pos_w[Y];

    //the covariance matrix of the Observation (JJT) must be rotated as well:
    wRb[0][0] = wTb[0][0]; wRb[0][1] = wTb[0][1];
    wRb[1][0] = wTb[1][0]; wRb[1][1] = wTb[1][1];

    // // // compute observation cov (JJT) 
    // stereoJJT(roger, gammaR, gammaL, &cov_b);
    K = (2*BASELINE)/(sin(gammaR-gammaL)*sin(gammaR-gammaL));

    jacobi_m[0][0] = K * pow(cos(gammaR),2); 
    jacobi_m[0][1] = -K * pow(cos(gammaL),2); 
    jacobi_m[1][0] = K * sin(gammaR) * cos(gammaR);
    jacobi_m[1][1] = -K * sin(gammaL) * cos(gammaL);

    matrix_transpose(2,2,jacobi_m,jacobi_transpose);
    matrix_mult(2,2,jacobi_m,2,jacobi_transpose,cov_b);
    // // and rotate it into world coordinates ... [cov]_w = wRb [cov]_b wRb^T
  
    matrix_transpose(2, 2, wRb, bRw);
    matrix_mult(2, 2, wRb, 2, cov_b, mat2X2);
    matrix_mult(2, 2, mat2X2, 2, bRw, obs->cov); 

    // and scale it experimentally, you can use enter_params( ) for scale
    obs->cov[0][0] *= scale; 
    obs->cov[0][1] *= scale;                                                                                                                                                                                                                                                 
    obs->cov[1][0] *= scale; 
    obs->cov[1][1] *= scale;
    
    // roger->drawing_request.obs_number ++; 
  } 
} 
 
int obs_number=0;
void project5_control(roger, time)
Robot* roger;
double time;
{ 
  // stereo_observation(roger, time); 
  roger->drawing_request.obs_number = obs_number; 
} 

/************************************************************************/
void project5_reset(roger)
Robot* roger; 
{ } 
// prompt for and read user customized input values
void project5_enter_params() 
// Robot* roger; 
{
  // printf("Project 5 enter_params called. \n");
   obs_number +=1;  
}
 
//function called when the 'visualize' button on the gui is pressed
void project5_visualize(roger)
Robot* roger;
{ 
  void draw_observation();
  Observation obs;
  stereo_observation(roger, &obs);
  draw_observation(roger, obs);
  
}



