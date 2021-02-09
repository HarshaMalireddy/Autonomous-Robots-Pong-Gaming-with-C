/*************************************************************************/
/* File:        project11.c                                              */
/* Description: User project #11 - empty project directory for project   */
/*              development                                              */
/* Date:        01-2015                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

void project11_control(roger, time)
Robot* roger;
double time;
{ }

/************************************************************************/
void project11_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project11_enter_params() 
{
  printf("Project 11 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project11_visualize(roger)
Robot* roger;
{ }
