/*
 *  control.c
 *  
 *
 *  Created by Shiraj Sen on 3/31/13.
 *  Updated to comply with new gcc settings 4/2014 by Mike Lanighan.
 *  Updated to support the new two-mode version of the simulator 4/2018 by Sadegh Rabiee
 *  Copyright 2013 University of Massachusetts Amherst. All rights reserved.
 *
 */


#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"

#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

int init_control_flag = TRUE;
void project4_control(), project4_reset(), project6_reset(), project6_control();
void inv_arm_kinematics();

// Visualization helper functions ******
// observations in world coordinates
void draw_observation(roger, obs)
Robot* roger;
Observation obs;
{
  int i, j;
  int request_num = roger->drawing_request.obs_number;

  if (request_num < NMAX_OBS) {
    for (i = 0; i < 2; i++) {
      roger->drawing_request.obs_args[request_num].obs.pos[i] = obs.pos[i];
    }

    for (i = 0; i < 2; i++) {
      for (j = 0; j < 2; j++) {
        roger->drawing_request.obs_args[request_num].obs.cov[i][j] = obs.cov[i][j];
      }
    }

    roger->drawing_request.obs_args[request_num].obs.time = obs.time;
    roger->drawing_request.obs_number++;

  }
}

void draw_estimate(roger, scale, est)
Robot* roger;
double scale;
Estimate est;
{
  int i, j;
  int request_num = roger->drawing_request.est_number;

  if (request_num < NMAX_EST) {
    roger->drawing_request.est_args[request_num].scale = scale;

    for (i = 0; i < 4; i++) {
      roger->drawing_request.est_args[request_num].est.state[i] = est.state[i];
    }

    for (i = 0; i < 4; i++) {
      for (j = 0; j < 4; j++) {
        roger->drawing_request.est_args[request_num].est.cov[i][j] = est.cov[i][j];
      }
    }

    roger->drawing_request.est_args[request_num].est.time = est.time;
    roger->drawing_request.est_number++;
  }
}

void draw_history(roger)
Robot* roger;
{
  roger->drawing_request.draw_history = TRUE;
}

void draw_streamline(roger)
Robot* roger;
{
  roger->drawing_request.draw_streamline = TRUE;
}

void Teleoperation_Cartesian_input_arms(roger, x, y, button)
Robot* roger;
double x;
double y;
int button;
{
  int limb = LEFT;

  printf("Arm goal input - x: %4.3f, y: %4.3f - button: %d\n", x, y, button);

  if (button == LEFT_BUTTON) {
    limb = LEFT;
    printf("world frame input for LEFT arm x=%6.4lf y=%6.4lf\n", x, y);
  }
  else if (button == RIGHT_BUTTON) {
    limb = RIGHT;
    printf("world frame input for RIGHT arm x=%6.4lf y=%6.4lf\n", x, y);
  }
  else return;
  inv_arm_kinematics(roger, limb, x,y); //PROJECT #2 - ArmKinematics/project2.c
}

void Teleoperation_Cartesian_input_base(roger, x, y, button)
Robot* roger;
double x;
double y;
int button;
{
  double dx, dy, theta;

  roger->base_setpoint[X] = x;
  roger->base_setpoint[Y] = y;
  dx = x - roger->base_position[X];
  dy = y - roger->base_position[Y];
  theta = roger->base_setpoint[THETA] = atan2(dy,dx);
  printf("world frame Base goal: x=%6.4lf y=%6.4lf theta=%6.4lf\n", x,y,theta);
}

//check if input is in configuration space area and return angles
int isConfigurationInput(x, y, zoom, q1, q2, side) 
double x, y;
float zoom;
double *q1, *q2;
int *side;
{
  double range = 0.0;

  *q1 = 0.0;
  *q2 = 0.0;
  *side = LEFT;

  //printf("Conf space check: %6.4lf, %6.4lf \n", x, y);

  //check if click is in configuration space area
  //X-Direction
  if (x < D2WX_DEV(zoom, T12LD(zoom,T1_MIN)) || (x > D2WX_DEV(zoom, T12LD(zoom,T1_MAX)) && 
      x < D2WX_DEV(zoom, T12RD(zoom,T1_MIN))) || x > D2WX_DEV(zoom, T12RD(zoom,T1_MAX))) {
    //printf("x-location out of bounds!!! %6.4lf \n", x);
    return FALSE;
  }
  //Y-Direction
  else if (y < D2WY_DEV(zoom, T22LD(zoom,T2_MIN)) || y > D2WY_DEV(zoom, T22LD(zoom,T2_MAX)) ) {
    //printf("y-location out of bounds!!! %6.4lf\n", y);
    return FALSE;
  }

  //calculate joint angles from click locations
  //left arm
  if (x < D2WX_DEV(zoom, T12LD(zoom,T1_MAX))) {
    *side = LEFT;
    range = D2WX_DEV(zoom, T12LD(zoom,T1_MAX)) - D2WX_DEV(zoom, T12LD(zoom,T1_MIN));
    *q1 = (x - (D2WX_DEV(zoom, T12LD(zoom,T1_MIN)) + range/2.0)) / (range/2.0) * M_PI;
    range = D2WY_DEV(zoom, T22LD(zoom,T2_MAX)) - D2WY_DEV(zoom, T22LD(zoom,T2_MIN));
    *q2 = (y - (D2WY_DEV(zoom, T22LD(zoom,T2_MIN)) + range/2.0)) / (range/2.0) * M_PI;
  }
  //right arm
  else {
    *side = RIGHT;
    range = D2WX_DEV(zoom, T12RD(zoom,T1_MAX)) - D2WX_DEV(zoom, T12RD(zoom,T1_MIN));
    *q1 = (x - (D2WX_DEV(zoom, T12RD(zoom,T1_MIN)) + range/2.0)) / (range/2.0) * M_PI;
    range = D2WY_DEV(zoom, T22RD(zoom,T2_MAX)) - D2WY_DEV(zoom, T22RD(zoom,T2_MIN));
    *q2 = (y - (D2WY_DEV(zoom, T22RD(zoom,T2_MIN)) + range/2.0)) / (range/2.0) * M_PI;
  }
  return TRUE;
}

//check if input is in cartesian space area
int isCartesianInput(x_input, y_input, environment, x, y) 
double x_input, y_input;
int environment;
double *x, *y;
{
  if (environment == ARENA) {
    if (x_input<MIN_X || x_input>MAX_X || y_input<MIN_Y || y_input>MAX_Y) {
      //printf("Location out of bounds!!!\n");
      return FALSE;
    }
  } else {
    if (x_input<MIN_X_DEV || x_input>MAX_X_DEV || y_input<MIN_Y_DEV || y_input>MAX_Y_DEV) {
      //printf("Location out of bounds!!!\n");
      return FALSE;
    }
  }
  
  *x = x_input;
  *y = y_input;

  return TRUE;
}

/***********************************************************************/
/** Input modes --- update the setpoint data structure:            *****/ 
/**    JOINT_ANGLE_INPUT mode: mouse(q1,q2) -> arm (q1, q2) ref    *****/
/**      BASE_GOAL_INPUT mode: mouse(x,y) -> base (x,y) ref	   *****/
/**      ARM_GOALS_INPUT mode: mouse(x,y) -> arm (x,y) ref  	   *****/
/**        BALL_POSITION mode: mouse(x,y) -> red ball (x,y)        *****/
/**           MAP_EDITOR mode: left button(x,y) -> obstacle        *****/
/**                            right button(x,y) -> goal           *****/
/***********************************************************************/
void update_setpoints(roger)
Robot * roger;
{
  //cartesian coordinates
  double x, y;
  //configuration coordinates
  double q1, q2;
  //body side
  int body_side = 0;

  //*******************************************************************
  //***** process enter param request based on selected input mode ****
  //*******************************************************************
  if (roger->enter_param_event) {
    switch(roger->control_mode) {
      case PROJECT2: project2_enter_params(); break;
      case PROJECT3: project3_enter_params(); break;
      case PROJECT4: project4_enter_params(); break;
      case PROJECT5: project5_enter_params(); break;
      case PROJECT6: project6_enter_params(); break;
      case PROJECT7: project7_enter_params(); break;
      case PROJECT8: project8_enter_params(); break;
      case PROJECT9: project9_enter_params(); break;
      case PROJECT10: project10_enter_params(); break;
      case PROJECT11: project11_enter_params(); break;
      default:
      project1_enter_params();
    }
    roger->enter_param_event = FALSE;
  }

  //*******************************************************************
  //***** process button input based on selected input mode ***********
  //*******************************************************************
  if (roger->button_event) {

    switch(roger->input_mode) {
      case JOINT_ANGLE_INPUT:
        if (roger->environment == ARENA) {
          break;
        }

        //check if inputs are valid
        if (isConfigurationInput(roger->button_reference[X],
                   roger->button_reference[Y],
                   roger->graphics.zoom, &q1, &q2,
                   &body_side) == FALSE) {
          printf("Invalid joint input\n");
          break;
        }
        
        if (roger->control_mode == PROJECT1) { // GUI inputs to Motor Units
          //left mouse button -> arms
          if (roger->button_event == LEFT_BUTTON) {
            printf("q1=%6.4lf q2=%6.4lf\n", q1, q2);
            
            roger->arm_setpoint[body_side][0] = q1;
            roger->arm_setpoint[body_side][1] = q2;
          }
          //right mouse button -> eyes
          else if (roger->button_event == RIGHT_BUTTON) {
            printf("q1=%6.4lf \n", q1);
            roger->eyes_setpoint[body_side] = q1;
          }
        }
        break;


      case BASE_GOAL_INPUT:
        if (roger->environment == ARENA) {
          break;
        }

        //check if inputs are valid
        if (isCartesianInput(roger->button_reference[X], 
           roger->button_reference[Y], roger->environment, &x, &y) == FALSE) {
          break;
        }
         
        if (roger->control_mode == PROJECT1)
          Teleoperation_Cartesian_input_base(roger, x, y, roger->button_event);
        break;

      case ARM_GOAL_INPUT:
        if (roger->environment == ARENA) {
          break;
        }
        // BUTTON INTERFACE - ARM TELEOPERATOR - CARTESIAN CONTROL
        //check if inputs are valid
        if (isCartesianInput(roger->button_reference[X],
           roger->button_reference[Y], roger->environment, &x,&y) == FALSE) {
          break;
        }
        //      printf("inside arm_goal_input case");
        if ((roger->control_mode == PROJECT1) ||
            (roger->control_mode == PROJECT2))
          Teleoperation_Cartesian_input_arms(roger, x, y, roger->button_event);
        break;
    }
    roger->button_event = FALSE;
  }



	//*******************************************************************
	//******** perform control based on selected control mode ***********
	//*******************************************************************
	    
  switch(roger->control_mode) {
     case PROJECT2:
       project2_control(roger, roger->simtime);
       project2_visualize(roger);
       break;
     case PROJECT3:
       project3_control(roger, roger->simtime);
       project2_visualize(roger);
       break;
     case PROJECT4:
       project4_control(roger,roger->simtime);
       project4_visualize(roger);
       break;
     case PROJECT5:
       project5_control(roger, roger->simtime);
       project5_visualize(roger);
       break;
     case PROJECT6:
       project6_control(roger, roger->simtime);
       project6_visualize(roger);
       break;
     case PROJECT7:
       project7_control(roger, roger->simtime);
       project7_visualize(roger);
       break;
     case PROJECT8:
       project8_control(roger, roger->simtime);
       project8_visualize(roger);
       break;
     case PROJECT9:
       project9_control(roger, roger->simtime);
       project9_visualize(roger);
     case PROJECT10:
       project10_control(roger, roger->simtime);
       project10_visualize(roger);
     case PROJECT11:
       project11_control(roger, roger->simtime);
       project11_visualize(roger);
       break;
     default:
       break;
  }
}


// initialize all setpoints to reflect the current posture, set the Cartesian
//    button_reference as appropriate for the new mode (base, left arm,
//       right arm, stereo head, harmonic function, integrated app)
void initialize_control_mode(roger)
Robot * roger;
{
  double wTb[4][4],ref_w[4],ref_b[4];
  int i,j;
	
  // define all setpoints to current positions and zero velocities
  roger->base_setpoint[X] = roger->base_position[X];// + BASE_CONTROL_OFFSET*cos(roger->base_position[THETA]);
  roger->base_setpoint[Y] = roger->base_position[Y];// + BASE_CONTROL_OFFSET*sin(roger->base_position[THETA]);
  roger->base_setpoint[THETA] = roger->base_position[THETA];
	
  roger->arm_setpoint[LEFT][0] = roger->arm_theta[LEFT][0];
  roger->arm_setpoint[LEFT][1] = roger->arm_theta[LEFT][1];
  roger->arm_setpoint[RIGHT][0] = roger->arm_theta[RIGHT][0];
  roger->arm_setpoint[RIGHT][1] = roger->arm_theta[RIGHT][1];
	
  roger->eyes_setpoint[LEFT] = roger->eye_theta[LEFT];
  roger->eyes_setpoint[RIGHT] = roger->eye_theta[RIGHT];
	
  // call the reset method of the last mode --> clear everything you modified
  switch ((roger->control_mode + N_CONTROL_MODES - 1) % N_CONTROL_MODES) {
    // "+ N_CONTROL_MODES" needed as modulo of negative numbers incorrect
     case PROJECT3:
       project3_reset(roger);
       break;
     case PROJECT4:
       project4_reset(roger);
       break;
     case PROJECT5:
       project5_reset(roger);
       break;
     case PROJECT6:
       project6_reset(roger);
       break;
     case PROJECT7:
       project7_reset(roger);
       break;
     case PROJECT8:
       project8_reset(roger);
       break;
     case PROJECT9:
       project9_reset(roger);
       break;
     case PROJECT10:
       project10_reset(roger);
       break;
     case PROJECT11:
       project11_reset(roger);
       break;
     default:
       break;
  }
  init_control_flag = FALSE; 
}
