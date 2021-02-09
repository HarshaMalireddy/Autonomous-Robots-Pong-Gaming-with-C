/*********************************************************************
 *  File:         matrix_math.h
 *  Description:  Definitions of functions to perform matrix math
 *  Author:       Mike Lanighan
 *  Date:         2014
 *********************************************************************/
#include <math.h>
#include <stdio.h>

void matrix_copy(), matrix_mult(), matrix_transpose(), matrix_add();
void matrix_subtract(), pseudoinverse(), nullspace();
int matrix_invert();

// Homogeneous Transform operators
void HT_invert(), construct_wTb();





