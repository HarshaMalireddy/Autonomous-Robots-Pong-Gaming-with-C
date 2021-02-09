/************************************************************************/
/* the probability distribution for the red ball                        */
/*              Pr(theta_base | TRACK(red ball) = CONVERGED)            */
/*         global variable in the project 3 scope                       */
/************************************************************************/
#define NHEADINGS 64
#define NH        NHEADINGS
#define RAD_PER_HEADING_BIN (2.0*M_PI/NHEADINGS)
/* integer Bin (0:64) to double Heading (-pi:pi)                        */
#define B2H(num) ( ((double)num+0.5)*RAD_PER_HEADING_BIN - M_PI )
/* double Heading (-pi:pi) to integer Bin (0:64)                        */
#define H2B(num) ( (int)((num+M_PI)/RAD_PER_HEADING_BIN) )
/************************************************************************/

