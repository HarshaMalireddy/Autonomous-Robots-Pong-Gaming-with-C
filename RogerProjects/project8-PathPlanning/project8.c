/*************************************************************************/
/* File:        project8.c                                               */
/* Description: User project #8 - harmonic function code                 */
/* Date:        12-2014                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"

#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

void sor()
{ }

double compute_gradient(x, y, roger, grad)
     double x, y;
     Robot * roger;
     double grad[2]; // grad = [ d(phi)/dx  d(phi)/dy ]                              
{
  return(0.0);
}

/************************************************************************/
void project8_control(roger, time)
Robot *roger;
double time;
{ }

/************************************************************************/
void project8_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project8_enter_params() 
{
  printf("Project 8 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project8_visualize(roger)
Robot * roger;
{ }

