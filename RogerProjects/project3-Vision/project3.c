/*************************************************************************/
/* File:        project3.c                                               */
/* Description: User project #3 - empty project directory for project    */
/*              development                                             */
/* Date:        01-2015                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

int average_red_pixel(roger, ul, ur)
Robot* roger;
double *ul; 
double *ur; 
{
  int left_red_count = 0;
  int right_red_count = 0;

  for(int e = 0; e < 2; e++) {
    for(int i = 0; i < 128; i++) {
      if(roger->image[e][i][0] == 255) {
        if(e==0) { 
          *ul += i; 
          left_red_count += 1;
        } 
        else if(e==1) {
          *ur += i;
          right_red_count += 1;
        }  
      } 
    }
  }

  // if(red is detected in both images) estimate image coordinates ul and ur;
  if(left_red_count > 0 && right_red_count > 0) {
    
    *ul /= left_red_count;
    *ur /= right_red_count; 

    return(TRUE);
  } 
  else return(FALSE);  
 
}   

void project3_control(roger, time)
Robot* roger;
double time; 
{ 
    double ul = 0.0;  
    double ur = 0.0;
 
    double oc_left_error = 0.0;
    double oc_right_error = 0.0;
    
    // FILE *fl_two_left;  
    // FILE *fl_two_right;
    // fl_two_left = fopen("./project3-Vision/4.3_two_left.csv", "a"); 
    // fl_two_right = fopen("./project3-Vision/4.3_two_right.csv", "a");
    
    FILE *fl_three_left;  
    FILE *fl_three_right;
    fl_three_left = fopen("./project3-Vision/4.3_three_left.csv", "a"); 
    fl_three_right = fopen("./project3-Vision/4.3_three_right.csv", "a");
    
       
    if(average_red_pixel(roger, &ul, &ur)) { 
 
      oc_left_error = atan2((double) (ul-63), (double) FOCAL_LENGTH);
      oc_right_error = atan2((double) (ur-63), (double) FOCAL_LENGTH);   

      roger->eyes_setpoint[0] = roger->eye_theta[0] + oc_left_error; 
      roger->eyes_setpoint[1] = roger->eye_theta[1] + oc_right_error;  

      // fprintf(fl_two_left, "%lf,%lf\n", time, oc_left_error);
      // fprintf(fl_two_right, "%lf,%lf\n", time, oc_right_error);
 
      fprintf(fl_three_left, "%lf,%lf\n", time, oc_left_error);
      fprintf(fl_three_right, "%lf,%lf\n", time, oc_right_error);
    }     
    // fclose(fl_two_left);
    // fclose(fl_two_right); 
 
    fclose(fl_three_left); 
    fclose(fl_three_right);
}

/************************************************************************/
void project3_reset(roger)

Robot* roger;
{ }
 
// prompt for and read user customized input values
void project3_enter_params() 
{
  printf("Project 4 enter_params called. \n"); 
}

//function called when the 'visualize' button on the gui is pressed
void project3_visualize(roger)
Robot* roger;
{ }

