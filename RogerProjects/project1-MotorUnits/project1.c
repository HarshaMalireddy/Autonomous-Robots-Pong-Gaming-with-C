/*************************************************************************/
/* File:        project1.c                                               */
/* Description: PD control analogs of the biological motor units for     */
/*              every degree of freedom: eyes, arms, base rotate and     */
/*              base translate --- motor units execute every simulated   */
/*              millisecond and are never disengaged. Higher-level       */
/*              control applications submit sequences of setpoints to    */
/*              combinations of motorunits.                              */
/* Date:        1-2015                                                   */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"
#include "matrix_math.h"

void update_setpoints();

/*************************************************************************/
/* PROJECT #1 - COMPLETE THE FOLLOWING CONTROL PROCEDURES                */
// gains for the PD controllers EYES
double Kp_eye = 1.0;
double Kd_eye = 0.017;   
double passive_Kd_eye = 0.001; 

// ARMS 
double Kp_arm =  80.0;
double Kd_arm =  4.0;
double passive_Kd_arm = 1.0;

// int started = 0; 
// double angle = 0.0; 

// BASE TRANSLATION 
double Kp_base_trans = 100.0; 
double Kd_base_trans = 26.2;  
double passive_Kd_base_trans = 2.0; 
  
// BASE ROTATION 
double Kp_base_rot =  118.0;  
double Kd_base_rot =  6.2;  
double passive_Kd_base_rot = 1.0; 
  
// double global_eyesetpoint = 0.0; 
/*************************************************************************/

/* PROJECT #1.1 - PD CONTROLLER FOR THE EYES                             */
/* setpoints are joint angle values in radians for the eyes              */
void PDController_eyes(roger, time)
Robot * roger;  
double time;
{  
  int i;
  double theta_error, theta_dot_error;  
  
  // FILE *fl;   
  // fl = fopen("./project1-MotorUnits/data_under.csv", "a");
  // roger->eyes_setpoint[0] = 1; 
   
  for (i = 0; i < NEYES; i++) {
    theta_error = roger->eyes_setpoint[i] - roger->eye_theta[i];
    theta_dot_error = 0.0 - roger->eye_theta_dot[i];
    //printf("Eye: %d \t",i); 
    if (ACTUATE_EYES) { 
      // REPLACE THE FOLLOWING LINE WITH THE PD CONTROLLER FOR THE EYE
      // USING YOUR GAINS 
 
      // torque = -Kp * theta_dot - Kd * (current angle - goal angle)
      roger->eye_torque[i] = Kp_eye * theta_error + Kd_eye * theta_dot_error;
      
      // if(i == 0) { 
      //   printf("%lf\t %lf\n", time, theta_error); 
      //   fprintf(fl, "%lf,%lf\n", time, theta_error);
      // } 
     
    }  
    else roger->eye_torque[i] = passive_Kd_eye*theta_dot_error;
  } 
  // fclose(fl); 

} 

/* PROJECT #1.2 - PD CONTROLLER FOR THE ARMS                             */
/* setpoints - joint angles in radians for the shoulders and elbows      */
void PDController_arms(roger, time)
Robot * roger;
double time;
{
  int i, j;
  double theta_error, theta_dot_error; 

  // FILE *fl_arms; 
  // fl_arms = fopen("./project1-MotorUnits/arms_data_q2.csv", "a");

  //  roger->arm_setpoint[0][0] = angle;
  //  roger->arm_setpoint[0][1] = angle;  

  for (i=LEFT; i<=RIGHT; ++i) {
    for (j=0; j<NARM_JOINTS; ++j) { 
       
      theta_error = roger->arm_setpoint[i][j] - roger->arm_theta[i][j];
      theta_dot_error = 0.0 - roger->arm_theta_dot[i][j];
   
 
      while (theta_error > M_PI) theta_error -= 2.0 * M_PI;
      while (theta_error < -M_PI) theta_error += 2.0 * M_PI;
 
      // tune kp_arm and kd_arm by changing their value using enter_params()
      if (ACTUATE_ARMS) {
      // REPLACE THE FOLLOWING LINE WITH THE PD CONTROLLER FOR THE ARM
      // USING YOUR GAINS
	        roger->arm_torque[i][j] = Kp_arm * theta_error + Kd_arm * theta_dot_error;
          
          // if(started == 1 && ((i==0 && j==0) || (i==0 && j==1))) { 
          //     printf("%lf\t %lf\n", time, theta_error); 
          //     fprintf(fl_arms, "%lf,%lf\n", time, theta_error);
          // }  
  
      } 
      else { 
	        roger->arm_torque[i][j] = passive_Kd_arm * theta_dot_error;
      }
    }
  }
  // fclose(fl_arms); 
} 

/* Base PD controller, Cartesian reference */
double PDBase_translate(roger, time) 
Robot * roger;
double time;
{ 
  double Fx, error[2], trans_error, trans_vel;
  
  // #1
  // roger->base_setpoint[X] = 0;  
  // roger->base_setpoint[Y] = -0.75;   
   
  // #2
  // roger->base_setpoint[X] = 0.75;  
  // roger->base_setpoint[Y] = 0;    
     
  // Rx = (Xref − Xact)    
  error[X] = roger->base_setpoint[X] - roger->base_position[X]; 
  // Ry = (Yref − Yact)  
  error[Y] = roger->base_setpoint[Y] - roger->base_position[Y]; 

  // Change in Distance: 
  trans_error = error[X]*cos(roger->base_position[THETA]) +
    error[Y]*sin(roger->base_position[THETA]); 

  // Change in Velocity:  
  trans_vel = roger->base_velocity[X]*cos(roger->base_position[THETA]) +
    roger->base_velocity[Y]*sin(roger->base_position[THETA]);

  FILE *fl_x;  
  FILE *fl_y;   
 
  fl_x = fopen("./project3-Vision/fl_x.csv", "a"); 
  fl_y = fopen("./project3-Vision/fl_y.csv", "a");

  if (ACTUATE_BASE) {
    // REPLACE THE FOLLOWING LINE WITH THE PD CONTROLLER FOR THE BASE
    // USING YOUR GAINS

    // Fx = (Kx * tx) - (Bx * t_dot_x) 
    Fx = (Kp_base_trans * trans_error) - (Kd_base_trans * trans_vel); 
    
    fprintf(fl_x, "%lf,%lf\n", time, error[X]);
    fprintf(fl_y, "%lf,%lf\n", time, error[Y]);
  }
  else {
    Fx = - passive_Kd_base_trans * trans_vel;
  }  
  
  fclose(fl_x); 
  fclose(fl_y); 

  return(Fx);
}  
 
/* Base PD controller, Cartesian reference */
double PDBase_rotate(roger, time) 
Robot * roger;
double time;
{
  double Mz, theta_error, theta_dot_error;
   // #3
  // roger->base_setpoint[THETA] = 0;      
     
  // Change in Angle: 
  theta_error = roger->base_setpoint[THETA] - roger->base_position[THETA];
  
  // Change in Angular Velocity: 
  theta_dot_error = 0.0 - roger->base_velocity[THETA];
   
  while (theta_error > M_PI) theta_error -= 2.0 * M_PI;
  while (theta_error < -M_PI) theta_error += 2.0 * M_PI;
  
  FILE *fl_theta; 

  fl_theta = fopen("./project3-Vision/fl_theta.csv", "a");
  
  if (ACTUATE_BASE) {
    // REPLACE THE FOLLOWING LINE WITH THE PD CONTROLLER FOR THE BASE
    // USING YOUR GAINS
 
    // Mz = (K_theta * theta_error) − (B_theta * theta_dot_error) 
    Mz = (Kp_base_rot * theta_error) + (Kd_base_rot * theta_dot_error); 
    
    fprintf(fl_theta, "%lf,%lf\n", time, theta_error);
  } 
  else { 
    Mz = passive_Kd_base_rot * theta_dot_error;
  }
  
  fclose(fl_theta); 

  return(Mz); 
} 

/* PROJECT #1.3 - PD CONTROLLER FOR THE BASE                             */
/* setpoints - (xy) location for translation heading in radians          */

/*   the base differential drive Jacobian:                               */
/*    |tau_l|     |Fx|      |  x_dot  |     |theta_dot_left |            */
/*    |     |= JT |  |      |         | = J |               |            */
/*    |tau_r|     |Mz|      |theta_dot|     |theta_dot_right|            */

double baseJT[2][2] = 
  {{(1.0/2.0), -(1.0/(2.0*R_AXLE))}, {(1.0/2.0), (1.0/(2.0*R_AXLE))}};

void PDController_base(roger, time)
Robot * roger;
double time;
{ 
  double Fx, Mz, PDBase_translate(), PDBase_rotate();

  //  Fx = PDBase_translate(roger,time); // translate along current heading
  //  Mz = 0.0;

  //  Fx = 0.0;                          // rotate in current footprint
  //  Mz = PDBase_rotate(roger,time);
 
  Fx = PDBase_translate(roger,time);     // translate and rotate 
  Mz = PDBase_rotate(roger,time);

  // integrated wheel torque control
  roger->wheel_torque[LEFT] = baseJT[0][0]*Fx + baseJT[0][1]*Mz;
  roger->wheel_torque[RIGHT] = baseJT[1][0]*Fx + baseJT[1][1]*Mz;
}   

/*************************************************************************/
/*       THE SIMULATOR EXECUTES control_roger() EVERY CONTROL CYCLE      */
/*                        *** DO NOT ALTER ***                           */
/*************************************************************************/
void control_roger(roger, time)
Robot * roger;
double time;
{
  update_setpoints(roger); // check_GUI_inputs(roger)
  // roger->eyes_setpoint[0] = global_eyesetpoint;
  // turn setpoint references into torques
  PDController_eyes(roger, time);
  PDController_arms(roger, time);
  PDController_base(roger, time);
  
} 

/*************************************************************************/
void project1_reset(roger)
Robot* roger;
{ }

/*************************************************************************/
/* prompt for and read user customized input values                      */
/*************************************************************************/
void project1_enter_params()
{
  // put anything in here that you would like to change at run-time
  // without re-compiling user project codes

  // printf("EYES: K=%6.4lf  B=%6.4lf\n", Kp_eye, Kd_eye); 
  // printf("EYES: enter 'K B'\n"); fflush(stdout);
  // scanf("%lf %lf", &Kp_eye, &Kd_eye);

  // printf("ARMS: enter 'q1 & q2'\n"); fflush(stdout);
  // scanf("%lf", &angle); 
  // started = 1;
  //change global variables 

  printf("Translate: K=%6.4lf  B=%6.4lf\n", Kp_base_trans, Kd_base_trans); 
  printf("Translate: enter 'K B'\n"); fflush(stdout);
  scanf("%lf %lf", &Kp_base_trans, &Kd_base_trans);   

  // printf("Rotate: K=%6.4lf  B=%6.4lf\n", Kp_base_rot, Kd_base_rot); 
  // printf("Rotate: enter 'K B'\n"); fflush(stdout);
  // scanf("%lf %lf", &Kp_base_rot &Kd_base_rot);
  
} 

/*************************************************************************/
// function called when the 'visualize' button on the gui is pressed            
void project1_visualize(roger)
Robot* roger;
{ }

