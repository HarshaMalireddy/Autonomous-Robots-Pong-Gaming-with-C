/*********************************************************************
 *  File:         matrix_math.c
 *  Description:  Functions to perform matrix math
 *  Author:       Mike Lanighan
 *  Date:         2014
 *********************************************************************/
#include <stdlib.h>
#include "matrix_math.h"

/* copy t1[rows][cols] into t2[rows][cols] */
void matrix_copy(int rows, int cols, double t1[rows][cols], 
		 double t2[rows][cols])
{
  int i,j;
  for (i=0; i<rows; ++i) {
    for (j=0; j<cols; ++j) {
      t2[i][j] = t1[i][j];
    }
  }
}

/** compute the product of two matrices                              */
/** IN: int m1, int n1, double t1[m1][n1], int n2, double t2[n1][n2] */
/** OUT: double result[m1][n2]                                       */
void matrix_mult(int m1, int n1, double t1[m1][n1],int n2, double t2[n1][n2],
		 double result[m1][n2])
{
  int i,j,k;
  for(i=0;i<m1;i++){
    for(j=0;j<n2;j++){
      result[i][j] = 0.0;
      for(k=0;k<n1;k++){
	result[i][j] += t1[i][k] * t2[k][j];
      }
    }
  }
}

/** transposes an m x n matrix                                       */
/** IN: int m, int n, double in[m][n]                                */
/** OUT: double out[n][m]                                            */
void matrix_transpose(int m, int n, double in[m][n], double out[n][m])
{
  int i,j;
  for (i=0; i<m; ++i) {
    for (j=0; j<n; ++j) {
      out[j][i] = in[i][j];
    }
  }
}

/** m x n matrix addition                                            */
/** IN: int m, int n, double in1[m][n], double in2[m][n]             */
/** OUT: double out[m][n]                                            */
void matrix_add(int m, int n, double in1[m][n], double in2[m][n],
		double out[m][n])
{
  int i,j;
  for (i=0; i<m; ++i) {
    for (j=0; j<n; ++j) {
      out[i][j] = in1[i][j] + in2[i][j];
    }
  }
}

/** m x n matrix subtraction (in1 - in2)                             */
/** IN: int m, int n, double in1[m][n], double in2[m][n]             */
/** OUT: double out[m][n] = in1 - in2                                */
void matrix_subtract(int m, int n, double in1[m][n], double in2[m][n],
		     double out[m][n])
{
  int i,j;
  for (i=0; i<m; ++i) {
    for (j=0; j<n; ++j) {
      out[i][j] = in1[i][j] - in2[i][j];
    }
  }
}

/** general Gauss Jordon matrix inverse of square matrix A[n][n]     */
/** IN: double * A, int n    OUT: double * A_inv                     */
/** returns 1 on success, 0 on failure.                              */
/** sample invocation: double* m = M[0];                             */
/**                    double* mi = MI[0];                           */
/**                    invert_M(m, 8 , mi);                          */
int matrix_invert(double* A, int n, double* A_inv)
{
  int i, j, iPass, imx, icol, irow;
  double det, temp, pivot, factor;
  double* ac = (double*)calloc(n*n, sizeof(double));
  det = 1.0;

  for (i = 0; i < n; i++) {
    for (j = 0; j < n; j++) {
      A_inv[n*i+j] = 0;
      ac[n*i+j] = A[n*i+j];
    }
    A_inv[n*i+i] = 1;
  }

  // The current pivot row is iPass.
  // For each pass, first find the maximum element in the pivot column.
  for (iPass = 0; iPass < n; iPass++) {
    imx = iPass;
    for (irow = iPass; irow < n; irow++) {
      if (fabs(A[n*irow+iPass]) > fabs(A[n*imx+iPass])) imx = irow;
    }
    // Interchange the elements of row iPass and row imx in both A and
    // A_inv
    if (imx != iPass) {
      for (icol = 0; icol < n; icol++) {
	temp = A_inv[n*iPass+icol];
	A_inv[n*iPass+icol] = A_inv[n*imx+icol];
	A_inv[n*imx+icol] = temp;

	if (icol >= iPass) {
	  temp = A[n*iPass+icol];
	  A[n*iPass+icol] = A[n*imx+icol];
	  A[n*imx+icol] = temp;
	}
      }
    }
    // The current pivot is now A[iPass][iPass].
    // The determinant is the product of the pivot elements.
    pivot = A[n*iPass+iPass];
    det = det * pivot;
    if (det == 0) {
      free(ac);
      return 0;
    }
    for (icol = 0; icol < n; icol++) {
      // Normalize the pivot row by dividing by the pivot element.
      A_inv[n*iPass+icol] = A_inv[n*iPass+icol] / pivot;
      if (icol >= iPass) A[n*iPass+icol] = A[n*iPass+icol] / pivot;
    }
    for (irow = 0; irow < n; irow++) {
      // Add a multiple of the pivot row to each row.  The multiple factor
      // is chosen so that the element of A on the pivot column is 0.
      if (irow != iPass) factor = A[n*irow+iPass];
      for (icol = 0; icol < n; icol++) {
	if (irow != iPass) {
	  A_inv[n*irow+icol] -= factor * A_inv[n*iPass+icol];
	  A[n*irow+icol] -= factor * A[n*iPass+icol];
	}
      }
    }
  }
  free(ac);
  return 1;
}

/** Moore-Penrose right pseudoinverse of A[m][n] n>m                  */
/** IN: int m, int n, double * A                                      */
/** OUT: double * A_pinv                                              */
void pseudoinverse(int m, int n, double A[m][n], double A_pinv[n][m])
{
  // compute the right pseudoinverse of A := A# = A^T(AA^T)^-1
  double AT[n][m], AAT[m][m], AAT_inv[m][m];
  matrix_transpose(m, n, A, AT);
  matrix_mult(m, n, A, m, AT, AAT);
  matrix_invert(&AAT[0][0], m, &AAT_inv[0][0]); // why is this necessary?
  matrix_mult(n, m, AT, m, AAT_inv, A_pinv);
}

/** the nullspace (annihilator) of a matrix A                         */
/** N(A) = (I - A#A)                                                  */
/** IN: int m, int n, double A[m][n], double A_pinv                   */
/** OUT: double N[m][m]                                               */
void nullspace(int m, int n, double A[m][n], double A_pinv[n][m], 
		double N[m][m])
{
  int i;

  // compute A#A
  matrix_mult( n, m, A_pinv, n, A, N);

  // compute I - A#A
  for (i=0; i<n; ++i) {
	N[i][i] = 1.0 - N[i][i];
  }
}

/*****************************************************************************/
/************     special methods for homogeneous transforms     *************/
/*  Author:       Rod Grupen                                                 */
/*  Date:         2-28-89                                                    */
/*****************************************************************************/

/* a particular HT (world-to-base) that comes up often                       */
void construct_wTb(base_pos, wTb)
double base_pos[3]; // (x,y,theta)
double wTb[4][4];
{
  double s0, c0;
  s0 = sin(base_pos[2]);
  c0 = cos(base_pos[2]);

  wTb[0][0] = c0;  wTb[0][1] = -s0; wTb[0][2] = 0.0; wTb[0][3] = base_pos[0];
  wTb[1][0] = s0;  wTb[1][1] = c0;  wTb[1][2] = 0.0; wTb[1][3] = base_pos[1];
  wTb[2][0] = 0.0; wTb[2][1] = 0.0; wTb[2][2] = 1.0; wTb[2][3] = 0.0;
  wTb[3][0] = 0.0; wTb[3][1] = 0.0; wTb[3][2] = 0.0; wTb[3][3] = 1.0;
}

/* produce the output inverted homogeneous transform for the input transform */
void HT_invert(in, out)            // project 2, 
double in[4][4], out[4][4];
{
  int i,j;
  for (i=0; i<3; ++i) {
    for (j=0; j<3; ++j) {
      out[i][j] = in[j][i];
    }
  }
  out[3][0] = out[3][1] = out[3][2] = 0.0; out[3][3] = 1.0;

  out[0][3] = -in[0][3]*in[0][0] - in[1][3]*in[1][0] - in[2][3]*in[2][0];
  out[1][3] = -in[0][3]*in[0][1] - in[1][3]*in[1][1] - in[2][3]*in[2][1];
  out[2][3] = -in[0][3]*in[0][2] - in[1][3]*in[1][2] - in[2][3]*in[2][2];
}

/**********************************************************************/
/* other dimensioned operators: matrix/vector copy, cofactors,        */
/*      determinant, vector addition/subtraction                      */
/**********************************************************************/

/*********************************************************************/
void invert_matrix22(in, out)
double in[2][2], out[2][2];
{
  double cof[2][2], det;
  int i,j;
  det = in[0][0]*in[1][1] - in[1][0]*in[0][1];

  out[0][0] = 1.0/det * in[1][1];
  out[0][1] = -1.0/det * in[0][1];
  out[1][0] = -1.0/det * in[1][0];
  out[1][1] = 1.0/det * in[0][0];
}

// only for 4D matrices
double determinant(x)
double x[4][4];
{
  double det;

  det = x[0][0]*(x[1][1]*x[2][2]*x[3][3] + x[1][2]*x[2][3]*x[3][1] 
		 + x[1][3]*x[2][1]*x[3][2] - x[3][1]*x[2][2]*x[1][3] 
		 - x[3][2]*x[2][3]*x[1][1] - x[3][3]*x[2][1]*x[1][2])
    - x[0][1]*(x[1][0]*x[2][2]*x[3][3] + x[1][2]*x[2][3]*x[3][0] 
	       + x[1][3]*x[2][0]*x[3][2] - x[3][0]*x[2][2]*x[1][3] 
	       - x[3][2]*x[2][3]*x[1][0] - x[3][3]*x[2][0]*x[1][2])
    + x[0][2]*(x[1][0]*x[2][1]*x[3][3] + x[1][1]*x[2][3]*x[3][0] 
	       + x[1][3]*x[2][0]*x[3][1] - x[3][0]*x[2][1]*x[1][3] 
	       - x[3][1]*x[2][3]*x[1][0] - x[3][3]*x[2][0]*x[1][1])
    - x[0][3]*(x[1][0]*x[2][1]*x[3][2] + x[1][1]*x[2][2]*x[3][0] 
	       + x[1][2]*x[2][0]*x[3][1] - x[3][0]*x[2][1]*x[1][2] 
	       - x[3][1]*x[2][2]*x[1][0] - x[3][2]*x[2][0]*x[1][1]);
  return(det);
}

void cofactors(x, cof)
double x[4][4], cof[4][4];
{
  int i,j;
  double minor[3][3];

  cof[0][0] = x[1][1]*x[2][2]*x[3][3] + x[1][2]*x[2][3]*x[3][1] 
    + x[1][3]*x[2][1]*x[3][2] - x[3][1]*x[2][2]*x[1][3] 
    - x[3][2]*x[2][3]*x[1][1] - x[3][3]*x[2][1]*x[1][2];
  cof[0][1] = -1.0*(x[1][0]*x[2][2]*x[3][3] + x[1][2]*x[2][3]*x[3][0] 
		    + x[1][3]*x[2][0]*x[3][2] - x[3][0]*x[2][2]*x[1][3] 
		    - x[3][2]*x[2][3]*x[1][0] - x[3][3]*x[2][0]*x[1][2]);
  cof[0][2] = x[1][0]*x[2][1]*x[3][3] + x[1][1]*x[2][3]*x[3][0] 
    + x[1][3]*x[2][0]*x[3][1] - x[3][0]*x[2][1]*x[1][3] 
    - x[3][1]*x[2][3]*x[1][0] - x[3][3]*x[2][0]*x[1][1];
  cof[0][3] = -1.0*(x[1][0]*x[2][1]*x[3][2] + x[1][1]*x[2][2]*x[3][0] 
		    + x[1][2]*x[2][0]*x[3][1] - x[3][0]*x[2][1]*x[1][2]
		    - x[3][1]*x[2][2]*x[1][0] - x[3][2]*x[2][0]*x[1][1]);
  
  cof[1][0] = -1.0*(x[0][1]*x[2][2]*x[3][3] + x[0][2]*x[2][3]*x[3][1] 
		    + x[0][3]*x[2][1]*x[3][2] - x[3][1]*x[2][2]*x[0][3] 
		    - x[3][2]*x[2][3]*x[0][1] - x[3][3]*x[2][1]*x[0][2]);
  cof[1][1] = x[0][0]*x[2][2]*x[3][3] + x[0][2]*x[2][3]*x[3][0] 
    + x[0][3]*x[2][0]*x[3][2] - x[3][0]*x[2][2]*x[0][3] 
    - x[3][2]*x[2][3]*x[0][0] - x[3][3]*x[2][0]*x[0][2];
  cof[1][2] = -1.0*(x[0][0]*x[2][1]*x[3][3] + x[0][1]*x[2][3]*x[3][0] 
		    + x[0][3]*x[2][0]*x[3][1] - x[3][0]*x[2][1]*x[0][3]
		    - x[3][1]*x[2][3]*x[0][0] - x[3][3]*x[2][0]*x[0][1]);
  cof[1][3] = x[0][0]*x[2][1]*x[3][2] + x[0][1]*x[2][2]*x[3][0] 
    + x[0][2]*x[2][0]*x[3][1] - x[3][0]*x[2][1]*x[0][2] 
    - x[3][1]*x[2][2]*x[0][0] - x[3][2]*x[2][0]*x[0][1];
  
  cof[2][0] = x[0][1]*x[1][2]*x[3][3] + x[0][2]*x[1][3]*x[3][1] 
    + x[0][3]*x[1][1]*x[3][2] - x[3][1]*x[1][2]*x[0][3] 
    - x[3][2]*x[1][3]*x[0][1] - x[3][3]*x[1][1]*x[0][2];
  cof[2][1] = -1.0*(x[0][0]*x[1][2]*x[3][3] + x[0][2]*x[1][3]*x[3][0] 
		    + x[0][3]*x[1][0]*x[3][2] - x[3][0]*x[1][2]*x[0][3] 
		    - x[3][2]*x[1][3]*x[0][0] - x[3][3]*x[1][0]*x[0][2]);
  cof[2][2] = x[0][0]*x[1][1]*x[3][3] + x[0][1]*x[1][3]*x[3][0] 
    + x[0][3]*x[1][0]*x[3][1] - x[3][0]*x[1][1]*x[0][3] 
    - x[3][1]*x[1][3]*x[0][0] - x[3][3]*x[1][0]*x[0][1];
  cof[2][3] = -1.0*(x[0][0]*x[1][1]*x[3][2] + x[0][1]*x[1][2]*x[3][0] 
		    + x[0][2]*x[1][0]*x[3][1] - x[3][0]*x[1][1]*x[0][2] 
		    - x[3][1]*x[1][2]*x[0][0] - x[3][2]*x[1][0]*x[0][1]);
  
  cof[3][0] = -1.0*(x[0][1]*x[1][2]*x[2][3] + x[0][2]*x[1][3]*x[2][1] 
		    + x[0][3]*x[1][1]*x[2][2] - x[2][1]*x[1][2]*x[0][3] 
		    - x[2][2]*x[1][3]*x[0][1] - x[2][3]*x[1][1]*x[0][2]);
  cof[3][1] = x[0][0]*x[1][2]*x[2][3] + x[0][2]*x[1][3]*x[2][0] 
    + x[0][3]*x[1][0]*x[2][2] - x[2][0]*x[1][2]*x[0][3] 
    - x[2][2]*x[1][3]*x[0][0] - x[2][3]*x[1][0]*x[0][2];
  cof[3][2] = -1.0*(x[0][0]*x[1][1]*x[2][3] + x[0][1]*x[1][3]*x[2][0] 
		    + x[0][3]*x[1][0]*x[2][1] - x[2][0]*x[1][1]*x[0][3] 
		    - x[2][1]*x[1][3]*x[0][0] - x[2][3]*x[1][0]*x[0][1]);
  cof[3][3] = x[0][0]*x[1][1]*x[2][2] + x[0][1]*x[1][2]*x[2][0] 
    + x[0][2]*x[1][0]*x[2][1] - x[2][0]*x[1][1]*x[0][2] 
    - x[2][1]*x[1][2]*x[0][0] - x[2][2]*x[1][0]*x[0][1];
}

void invert_matrix44(in, out)
double in[4][4], out[4][4];
{
  double cof[4][4], det;
  int i,j;
  //  double ans[4][4];

  // 1/det [cof]T

  det = determinant(in);
  //  printf("det=%lf\n", det);

  cofactors(in, cof);

  for (i=0; i<4; ++i) {
    for (j=0; j<4; ++j) {
      out[i][j] = cof[j][i]/det;
    }
  }
}
