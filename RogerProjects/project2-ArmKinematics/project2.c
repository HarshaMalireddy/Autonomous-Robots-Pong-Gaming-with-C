/*************************************************************************/
/* File: project2.c                                                      */
/* Description: Kinematic function for Arms                              */
/*                 fwd_arm_kinematics() - maps joint angles in the arm   */
/*                    into endpoint coordinates in the base frame        */
/*                 inv_arm_kinematics() - maps endpoint position for the */
/*                    specified arm in base frame into two possible joint*/
/*                    angle configurations and selects one of the two    */
/*                    writes arm cspace setpoints                        */
/* Date:        1-2015                                                 */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"
#include "matrix_math.h"

/*************************************************************************/
/*** PROJECT #2 - FORWARD KINEMATICS: MAP (THETA1, THETA2) TO (X,Y)_B  ***/
/***              INVERSE KINEMATICS: MAP (X,Y)_B TO (THETA1, THETA2)  ***/
/*************************************************************************/

double eu_distance; 
double ref_w[4] = {0,0,0,0};
// ref_w[0] = 0.0;
// ref_w[1] = 0.0;
// ref_w[2] = 0.0;
// ref_w[3] = 0.0;
// FILE *car_error; 

void fwd_arm_kinematics(roger, limb, x, y)
Robot * roger;
int limb;
double *x, *y; 
{ 

  double wTb[4][4], ref_b[4], ref_w_[4];
  double d_x = 0.5*cos(roger->arm_theta[limb][0]) + 0.5*cos(roger->arm_theta[limb][0] + roger->arm_theta[limb][1]);
  double d_y = 0.5*sin(roger->arm_theta[limb][0]) + 0.5*sin(roger->arm_theta[limb][0] + roger->arm_theta[limb][1]) + ARM_OFFSET;
  construct_wTb(roger->base_position, wTb);

  ref_b[0] = d_x;
  ref_b[1] = d_y;
  ref_b[2] = 0.0;
  ref_b[3] = 1.0;

  matrix_mult(4, 4, wTb, 1, ref_b, ref_w_);
  *x = ref_w_[0];
  *y = ref_w_[1]; 
}

// int inv_arm_kinematics(roger, limb, x, y, hold, right1_plus, right2_plus, left1_minus, left2_minus)
int inv_arm_kinematics(roger, limb, x, y)
Robot * roger;
int limb;
double x, y;
// int hold; 
// double right1_plus, right2_plus, left1_minus, left2_minus; 
{
  double wTb[4][4], bTw[4][4], ref_b[4];

  double r2, c2, s2_plus, s2_minus, theta2_plus, theta2_minus;
  double k1, k2_plus, k2_minus, alpha_plus, alpha_minus;
  double theta1_plus, theta1_minus;

  // input (x,y) is in world frame coordinates - map it into the base frame
  construct_wTb(roger->base_position, wTb);
  HT_invert(wTb,bTw); 

  ref_w[0] = x;
  ref_w[1] = y;
  ref_w[2] = 0.0;
  ref_w[3] = 1.0;

  matrix_mult(4, 4, bTw, 1, ref_w, ref_b);
  
  if (limb==LEFT) ref_b[Y] -= ARM_OFFSET;
  else ref_b[Y] += ARM_OFFSET;  
 
  // printf("x = %6.4lf  y = %6.4lf OUT OF REACH\n", ref_b[X], ref_b[Y]);

  r2 = pow(ref_b[X],2) + pow(ref_b[Y],2); 
  c2 = (r2 - pow(L_ARM1,2) - pow(L_ARM2,2))/(2*L_ARM1*L_ARM2); 
  
  if(c2 >= -1 && c2 <= 1) {
    
    s2_plus = pow(1-pow(c2,2),1/2); 
    s2_minus = -1 * pow((1-pow(c2,2)),1/2);
    
    theta2_plus = atan2(s2_plus, c2); 
    theta2_minus = atan2(s2_minus, c2);

    k1 = L_ARM1 + L_ARM2*c2; 
    k2_plus = L_ARM2*s2_plus;
    k2_minus = L_ARM2*s2_minus;

    alpha_plus  = atan2(k2_plus, k1);
    alpha_minus = atan2(k2_minus, k1);

    theta1_plus  = atan2(ref_b[Y], ref_b[X]) - alpha_plus;
    theta1_minus = atan2(ref_b[Y], ref_b[X]) - alpha_minus;  

    // if (limb == RIGHT && ref_b[X] >= 0) {
    //   roger->arm_setpoint[limb][0] = theta1_plus;
    //   roger->arm_setpoint[limb][1] = theta2_plus;
    // } 

    // else if (limb == RIGHT) {
    //   roger->arm_setpoint[limb][0] = theta1_minus;
    //   roger->arm_setpoint[limb][1] = theta2_minus;
    // }

    // else if (ref_b[X] >= 0) {
    //   roger->arm_setpoint[limb][0] = theta1_minus;
    //   roger->arm_setpoint[limb][1] = theta2_minus;
    // } 

    // else {
    //   roger->arm_setpoint[limb][0] = theta1_plus;
    //   roger->arm_setpoint[limb][1] = theta2_plus;
    // }

    if(limb == RIGHT) {
      // if(hold == FALSE) { 
        roger->arm_setpoint[limb][0] = theta1_plus;
        roger->arm_setpoint[limb][1] = theta2_plus;
      // }
      // else {
      //   right1_plus = theta1_plus;
      //   right2_plus = theta2_plus;
      // } 
    }  
    else {
      // if(hold == FALSE) {
        roger->arm_setpoint[limb][0] = theta1_minus;
        roger->arm_setpoint[limb][1] = theta2_minus;
      // }
      // else {
      //   left1_minus = theta1_minus;
      //   left2_minus = theta2_minus;
      // }
    }  
  
    return TRUE; 
  }
  
  else   
    return FALSE; 

}

/*************************************************************************/
/* project development interface - called by GUI                         */
/*************************************************************************/
/* executed automatically when                                           */
/* control mode = PROJECT2; input mode = BALL INPUTS                     */
void project2_control(roger, time)
Robot * roger;
double time;
{ 
  double armpos_x;  
  double armpos_y; 
  int limb = LEFT; 
  double ref_b[4];  
  double *x, *y;  
  
  fwd_arm_kinematics(roger, limb, &armpos_x, &armpos_y); 
  eu_distance = pow((pow(ref_w[X] - armpos_x,2) + pow(ref_w[Y] - armpos_y,2)), 0.5); 
  
  printf("%lf,%lf\n", time, eu_distance); 
  
  // FILE* car_error; 
  // car_error = fopen("./project5-StereoKinematics/car_error.csv", "a");
  // fprintf(car_error, "%lf,%lf\n", time, car_error);
  //fclose(car_error);  
}


void project2_reset(roger)
Robot* roger;
{ }

void project2_enter_params() 
{
  printf("Project 6 enter_params called. \n");
}

void project2_visualize(roger)
Robot* roger;
{ }


