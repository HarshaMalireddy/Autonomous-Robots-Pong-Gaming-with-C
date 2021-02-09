// /*************************************************************************/
// /* File:        project9.c                                               */
// /* Description: User project #9 - PONG                                   */
// /* Date:        12-2014                                                  */
// /*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

int lrRoger = 0; 
int ballSide = 0; 
int found = 0; 
// int default_arm = 0; 

// Sets Roger back to base postion 
void RETREAT(roger, time, base_arm_goals)
Robot* roger;
double time; 
double base_arm_goals[2][3]; 
{   
  if(lrRoger == 0) {
    base_arm_goals[0][0] = -4.0; 
    base_arm_goals[0][1] = 0.4; 
  }
  else {
    base_arm_goals[0][0] = 4.0; 
    base_arm_goals[0][1] = 0.4; 
  }
} 

// Blocks ball with body or arms to prevent ball from reaching the goal 
int BLOCK(roger, time, base_arm_goals)
Robot* roger;
double time;
double base_arm_goals[2][3]; 
{  
  double ul; 
  double ur;  
  double eu_distance;  
  Observation obs;     
  
  eu_distance = pow((pow(base_arm_goals[0][0] - roger->base_position[X],2) + pow(base_arm_goals[0][1] - roger->base_position[Y],2)), 0.5); 

   if(eu_distance <= 0.5) { 

    if((lrRoger == 1 && roger->base_position[X] > base_arm_goals[0][0]) || (lrRoger == 0 && roger->base_position[X] < base_arm_goals[0][0])) {
      inv_arm_kinematics(roger, 0, base_arm_goals[0][0], base_arm_goals[0][1]);
      inv_arm_kinematics(roger, 1, base_arm_goals[0][0], base_arm_goals[0][1]);  
      
      RETREAT(roger, time, base_arm_goals); 

      return CONVERGED; 
    }
    
    RETREAT(roger, time, base_arm_goals);
    
    return TRANSIENT;
  } 
   
  return NO_REFERENCE;
} 

// Sets proper base positions
int setpoint_filter(roger, time, base_arm_goals) 
Robot* roger;  
double time;  
double base_arm_goals[2][3]; 
{
  Observation obs;
  double ul = 0.4;   
  double ur = 0.4;  
 
  // Sets the base and eye angles to look to the center of the ball
  SearchTrack(roger, time);  
 
  // Gets the position of the ball in the (x,y) plane with (obs->pos[X], obs->pos[Y])
  stereo_observation(roger, &obs);   

  // CHASE(roger, time, base_arm_goals); 

  if(roger->base_position[X] < 0) lrRoger = 0; // 0 means dealing with left Roger 
  else lrRoger = 1; // 1 means dealing with right Roger 
  printf("Roger x : %lf Roger y: %lf lrRoger: %d\n", roger->base_position[X],  roger->base_position[Y], lrRoger); 
  // printf("base_arm_goals[0][0]: %lf base_arm_goals[0][1]: %lf lrRoger: %d\n", base_arm_goals[0][0], base_arm_goals[0][1]); 

  if(obs.pos[X] < 0) ballSide = 0; // 0 means ball is on left side 
  else if(obs.pos[X] > 0) ballSide = 1; // 1 means ball is on right side 
  else ballSide = -1; // -1 means ball is not seen by Roger or is in middle
  printf("Ball x: %lf Ball y: %lf ballSide: %d\n", obs.pos[X], obs.pos[Y], ballSide); 
  
  if(average_red_pixel(roger, &ul, &ur)) found = 1; // Roger spots ball 
  else found = 0; // Roger is seaching for ball 
  printf("Sees ball: %d ", average_red_pixel(roger, &ul, &ur), found); 
 
  if(found == 1) {  
    if((lrRoger == 0 && ballSide == 0) || (lrRoger == 1 && ballSide == 1)) {
      base_arm_goals[0][0] = obs.pos[X] - 0.4;
      base_arm_goals[0][1] = obs.pos[Y] - 0.4;  
      
      // if((lrRoger == 0 && roger->base_position[X] > obs.pos[X]) || (lrRoger == 1 && roger->base_position[X] < obs.pos[X]) || (obs.pos[X] = 0 && obs.pos[Y] == 0)) { 
        if(obs.pos[X] = 0 && obs.pos[Y] == 0) { 
          RETREAT(roger, time, base_arm_goals);  
      }
    } 
    else {   
      RETREAT(roger, time, base_arm_goals); 
    }  
  }   
  else {  
    RETREAT(roger, time, base_arm_goals);  
  }    
  
  // default_arm = 0; 
  
  if(lrRoger == ballSide && lrRoger == 0 && (base_arm_goals[0][0] > -0.9 || roger->base_position[X] > -0.5)) {
    // if(lrRoger == ballSide) 
    if(base_arm_goals[0][0] != -4.0)
      base_arm_goals[0][0] = -0.9;  
    // else 
    //   RETREAT(roger, time, base_arm_goals);  
  }
  else if(lrRoger == ballSide && lrRoger == 1 && (base_arm_goals[0][0] < 0.9 || roger->base_position[X] < 0.5)) {
    // if(lrRoger == ballSide)
    if(base_arm_goals[0][1] != 4.0)
      base_arm_goals[0][0] = 0.9;  
    // else  
    //   RETREAT(roger, time, base_arm_goals);  
  } 
  if(lrRoger == ballSide && base_arm_goals[0][1] < -1 || roger->base_position[Y] < -1) {
    // if(lrRoger == ballSide)
    if(base_arm_goals[0][1] != 0)
      base_arm_goals[0][1] = -1;     
    // else  
    //   RETREAT(roger, time, base_arm_goals);   
  }
  else if(lrRoger == ballSide && base_arm_goals[0][1] > 1 || roger->base_position[Y] > 1) {
    // if(lrRoger == ballSide)
    if(base_arm_goals[0][1] != 0)
      base_arm_goals[0][1] = 1;  
    // else 
    //   RETREAT(roger, time, base_arm_goals);  
  }   
  printf("Goal x: %lf Goal y: %lf\n", base_arm_goals[0][0], base_arm_goals[0][1]); 
  printf("Roger THETA: %lf\n\n", roger->base_position[THETA]); 
  return CONVERGED;   
} 

// Perform high quality ball strikes 
int PUNCH(roger, time, base_arm_goals)
Robot* roger;   
double time;  
double base_arm_goals[2][3]; 
{ 
  double ul;  
  double ur;   
  double eu_distance;   
  //Observation obs;     
   
  // eu_distance = pow((pow(base_arm_goals[0][0] - roger->base_position[X],2) + pow(base_arm_goals[0][1] - roger->base_position[Y],2)), 0.5);
  eu_distance = pow((pow((base_arm_goals[0][0] + 0.4) - roger->base_position[X],2) + pow((base_arm_goals[0][1] + 0.4) - roger->base_position[Y],2)), 0.5);
  
  if(eu_distance <= 0.8) {  
    // inv_arm_kinematics(roger, 0, base_arm_goals[0][0] + 0.4, base_arm_goals[0][1] + 0.4);
    // inv_arm_kinematics(roger, 1, base_arm_goals[0][0] + 0.4, base_arm_goals[0][1] + 0.4);
    if(lrRoger == 0 && lrRoger == ballSide) {  
      if(roger->base_position[X] <= base_arm_goals[0][0] + 0.4) {
        if(roger->base_position[THETA] > 0 && roger->base_position[THETA] <= 3.14) 
          inv_arm_kinematics(roger, 0, base_arm_goals[0][0] + 0.4, base_arm_goals[0][1] + 0.4);  
        // else if(roger->base_position[THETA] < 1.57 && roger->base_position[THETA] <= 3.14) 
        //   inv_arm_kinematics(roger, 1, base_arm_goals[0][0] + 0.4, base_arm_goals[0][1] + 0.4);  
        // else if(roger->base_position[THETA] <= 0 && roger->base_position[THETA] > -1.57) 
        //   inv_arm_kinematics(roger, 1, base_arm_goals[0][0] + 0.4, base_arm_goals[0][1] + 0.4);  
        else if(roger->base_position[THETA] <= 0 && roger->base_position[THETA] > -3.14) 
          inv_arm_kinematics(roger, 1, base_arm_goals[0][0] + 0.4, base_arm_goals[0][1] + 0.4);   
      }    
      else {  
        if(roger->base_position[THETA] > 0 && roger->base_position[THETA] <= 3.14) {
          inv_arm_kinematics(roger, 0, base_arm_goals[0][0] + 0.4, base_arm_goals[0][1] + 0.4);
          inv_arm_kinematics(roger, 1, base_arm_goals[0][0] + 0.4, base_arm_goals[0][1] + 0.4);
        }  
        // else if(roger->base_position[THETA] < 1.57 && roger->base_position[THETA] <= 3.14) 
        //   inv_arm_kinematics(roger, 0, base_arm_goals[0][0] + 0.4, base_arm_goals[0][1] + 0.4);  
        // else if(roger->base_position[THETA] <= 0 && roger->base_position[THETA] > -1.57) 
        //   inv_arm_kinematics(roger, 0, base_arm_goals[0][0] + 0.4, base_arm_goals[0][1] + 0.4);  
        else if(roger->base_position[THETA] <= 0 && roger->base_position[THETA] > -3.14) {
          inv_arm_kinematics(roger, 0, base_arm_goals[0][0] + 0.4, base_arm_goals[0][1] + 0.4);
          inv_arm_kinematics(roger, 1, base_arm_goals[0][0] + 0.4, base_arm_goals[0][1] + 0.4); 
        }  
      } 
    }    
    else if(lrRoger == 1 && lrRoger == ballSide) {
      if(roger->base_position[X] >= base_arm_goals[0][0] + 0.4) {
        if(roger->base_position[THETA] > 0 && roger->base_position[THETA] <= 3.14) 
          inv_arm_kinematics(roger, 1, base_arm_goals[0][0] + 0.4, base_arm_goals[0][1] + 0.4);  
        // else if(roger->base_position[THETA] < 1.57 && roger->base_position[THETA] <= 3.14) 
        //   inv_arm_kinematics(roger, 1, base_arm_goals[0][0] + 0.4, base_arm_goals[0][1] + 0.4);  
        // else if(roger->base_position[THETA] <= 0 && roger->base_position[THETA] > -1.57) 
        //   inv_arm_kinematics(roger, 0, base_arm_goals[0][0] + 0.4, base_arm_goals[0][1] + 0.4);  
        else if(roger->base_position[THETA] <= 0 && roger->base_position[THETA] > -3.14) 
          inv_arm_kinematics(roger, 0, base_arm_goals[0][0] + 0.4, base_arm_goals[0][1] + 0.4);   
      } 
      else { 
        if(roger->base_position[THETA] > 0 && roger->base_position[THETA] <= 3.14) {
          inv_arm_kinematics(roger, 0, base_arm_goals[0][0] + 0.4, base_arm_goals[0][1] + 0.4);
          inv_arm_kinematics(roger, 1 , base_arm_goals[0][0] + 0.4, base_arm_goals[0][1] + 0.4);
        }  
        // else if(roger->base_position[THETA] < 1.57 && roger->base_position[THETA] <= 3.14) 
        //   inv_arm_kinematics(roger, 0, base_arm_goals[0][0] + 0.3, base_arm_goals[0][1] + 0.3);  
        // else if(roger->base_position[THETA] <= 0 && roger->base_position[THETA] > -1.57) 
        //   inv_arm_kinematics(roger, 1, base_arm_goals[0][0] + 0.3, base_arm_goals[0][1] + 0.3);  
        else if(roger->base_position[THETA] <= 0 && roger->base_position[THETA] > -3.14) {
          inv_arm_kinematics(roger, 0, base_arm_goals[0][0] + 0.4, base_arm_goals[0][1] + 0.4);
          inv_arm_kinematics(roger, 1, base_arm_goals[0][0] + 0.4, base_arm_goals[0][1] + 0.4);
        }   
      } 
    }
    
    if(roger->ext_force[0][0] != 0.4 || roger->ext_force[0][1] != 0.4 || roger->ext_force[1][0] != 0.4 || roger->ext_force[1][1] != 0.4) {
      return CONVERGED; 
    }
 
    return TRANSIENT;  
  }   
       
  //RETREAT(roger, time, base_arm_goals);  
  
  return NO_REFERENCE;
}    
 

void CHASEPUNCHRETREAT(roger, time)
Robot* roger;
double time;  
{  
  double internal_state[2], base_arm_goals[2][3];  
  internal_state[0] = setpoint_filter(roger, time, base_arm_goals); 
  internal_state[1] = PUNCH(roger, time, base_arm_goals);
  // for N=2
  int state, return_state;
  state = internal_state[1]*3 + internal_state[0];  
  switch (state) {  
                                                             // TOUCH          CHASE
    case 0:                                                  // NO_REFERENCE - NO_REFERENCE
    case 1:                                                  // NO_REFERENCE -  TRANSIENT
    case 2:                                                  // NO_REFERENCE -  CONVERGED
      return_state = TRANSIENT; 
       // Left arm 
      roger->arm_setpoint[0][0] = 3.14; 
      roger->arm_setpoint[0][1] = -3.10;   
      // Right arm  
      roger->arm_setpoint[1][0] = -3.14; 
      roger->arm_setpoint[1][1] = 3.10;  
      roger->base_setpoint[X] = base_arm_goals[0][0]; 
      roger->base_setpoint[Y] = base_arm_goals[0][1]; 
      break; 
    case 3:                                                  //  TRANSIENT   - NO_REFERENCE
    case 4:                                                  //  TRANSIENT   -  TRANSIENT
    case 5:                                                  //  TRANSIENT   -  CONVERGED
      return_state = TRANSIENT;
      roger->base_setpoint[X] = base_arm_goals[0][0];
      roger->base_setpoint[Y] = base_arm_goals[0][1]; 
      break;
    case 6:                                                  //  CONVERGED   - NO_REFERENCE
    case 7:                                                  //  CONVERGED   -  TRANSIENT
    case 8:                                                  //  CONVERGED   -  CONVERGED
      return_state = CONVERGED;
      roger->base_setpoint[X] = base_arm_goals[0][0];
      roger->base_setpoint[Y] = base_arm_goals[0][1];
      break;  
    default: 
      break;   
  } 
} 

void PONG(roger, time)
Robot* roger;
double time;   
{  
  double internal_state[2], base_arm_goals[2][3];  
  internal_state[0] = setpoint_filter(roger, time, base_arm_goals); 
  internal_state[1] = PUNCH(roger, time, base_arm_goals);
  // for N=2
  int state, return_state;
  state = internal_state[1]*3 + internal_state[0];  
  switch (state) {  
                                                             // TOUCH          CHASE
    case 0:                                                  // NO_REFERENCE - NO_REFERENCE
    case 1:                                                  // NO_REFERENCE -  TRANSIENT
    case 2:                                                  // NO_REFERENCE -  CONVERGED
      return_state = TRANSIENT; 
       // Left arm 
      roger->arm_setpoint[0][0] = 3.14; 
      roger->arm_setpoint[0][1] = -3.10;   
      // Right arm  
      roger->arm_setpoint[1][0] = -3.14; 
      roger->arm_setpoint[1][1] = 3.10;  
      roger->base_setpoint[X] = base_arm_goals[0][0]; 
      roger->base_setpoint[Y] = base_arm_goals[0][1]; 
      break; 
    case 3:                                                  //  TRANSIENT   - NO_REFERENCE
    case 4:                                                  //  TRANSIENT   -  TRANSIENT
    case 5:                                                  //  TRANSIENT   -  CONVERGED
      return_state = TRANSIENT;
      roger->base_setpoint[X] = base_arm_goals[0][0];
      roger->base_setpoint[Y] = base_arm_goals[0][1]; 
      break;
    case 6:                                                  //  CONVERGED   - NO_REFERENCE
    case 7:                                                  //  CONVERGED   -  TRANSIENT
    case 8:                                                  //  CONVERGED   -  CONVERGED
      return_state = CONVERGED;
      roger->base_setpoint[X] = base_arm_goals[0][0];
      roger->base_setpoint[Y] = base_arm_goals[0][1];
      break;  
    default: 
      break;   
  } 
} 

void project9_control(roger, time)
Robot* roger;
double time; 
{ 
  // CHASEPUNCHRETREAT(roger, time);
  PONG(roger, time);
} 

/************************************************************************/
void project9_reset(roger)
Robot* roger;
{ }
 
// prompt for and read user customized input values
void project9_enter_params() 
{
  printf("Project 9 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project9_visualize(roger)
Robot* roger;
{ } 