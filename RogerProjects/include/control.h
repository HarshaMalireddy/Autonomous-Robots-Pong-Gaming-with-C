/**************************************************************************/
/* File:        control.h                                                 */
/* Description: all the compile time constants for use in the simulator   */
/* Author:      Rod Grupen                                                */
/* Date:        11-1-2009                                                 */
/**************************************************************************/
#define ACTUATE_BASE  1 // 1 => ACTUATED,  0 => UNACTUATED
#define ACTUATE_EYES  1 // 1 => ACTUATED,  0 => UNACTUATED
#define ACTUATE_ARMS  1 // 1 => ACTUATED,  0 => UNACTUATED
#define HISTORY       0 // 0 => draw current state, 1 => draw whole sequence

#define X             0
#define Y             1
#define XDOT          2
#define YDOT          3

#define THETA         2

#define NPRIMARY_COLORS 3
#define RED_CHANNEL   0
#define GREEN_CHANNEL 1
#define BLUE_CHANNEL  2

#define LEFT          0
#define RIGHT         1
#define TRUE          1
#define FALSE         0

/***** INVERSE KINEMATICS *************************************************/
#define OUT_OF_REACH     -1
#define IN_REACH          1

/***** PD CONTROL *********************************************************/
#define KP_BASE               75.0
#define KD_BASE               12.2
#define BASE_CONTROL_OFFSET    0.17

#define KP_ARM                80.0
#define KD_ARM                10.0

#define KP_EYE                 1.0
#define KD_EYE                 (sqrt(4.0*KP_EYE*I_EYE))

/***** CONTROL RETURN STATUS **********************************************/
// enum {
//   UNKNOWN = 0,
//   NO_REFERENCE,
//   TRANSIENT,
//   CONVERGED
// };

enum {
  NO_REFERENCE = 0,
  TRANSIENT,
  CONVERGED
};

/***** KALMAN FILTER PARAMETERS *******************************************/
#define SIGMA_OBS     0.01    // the scale of the observation covariance
#define SIGMA_PROCESS 10.0 // noise in the forward/process model   
#define OBS_DT        0.001 // same as servo and render rate

/***** SOR RELAXATION PARAMETERS ******************************************/
#define THRESHOLD              0.00000000000001
#define FREESPACE              1
#define OBSTACLE	           2
#define GOAL                   3
#define DILATED_OBSTACLE 	   4

#define SGN(x)   (((x) > 0) ? (1.0) : (-1.0))
#define SQR(x)   (((x)*(x)))
#define MIN(x,y) (((x)<(y)) ? (x) : (y))
#define MAX(x,y) (((x)>(y)) ? (x) : (y))

#define ARM_CSPACE_MAP     FALSE
#define NSTEPS 2000
#define MAX_FORCE 200

/*************************************************************************/
/* data structures that comprise the Robot interface to user control     */
/* applications (user file control.c)                                    */
#define NBINS 64  /* USER DEFINED - the number of nodes in each          */
                  /* dimension of all occupancy grids                    */

// Maximum number of draw_observation requests
#define NMAX_OBS 30

// Maximum number of draw_estimate requests
#define NMAX_EST 30

typedef struct _map {               /* DO NOT ALTER */
  int occupancy_map[NBINS][NBINS];
  double potential_map[NBINS][NBINS];
  int color_map[NBINS][NBINS];
} Map;

typedef struct _setpoint {    /* DO NOT ALTER */
  double base[2];             /* (x,y) of the base in world frame */
  double arm[NARMS][NARM_JOINTS]; /* arm joint angles */
  double eyes[NEYES];         /* eye verge angle  relative to base frame */
} SetPoint;

typedef struct _graphics {
  float zoom;
} Graphics;

// ************* Drawing requests ********************

typedef struct _observation {    /* DO NOT ALTER */
  double pos[2];                 /* [X Y] */
  double cov[2][2];              /*  J J^T */
  double time;                   /* time stamp for the observation */
} Observation;

typedef struct _estimate {    /* DO NOT ALTER */
  double state[4];            /* [X Y XDOT YDOT] */
  double cov[4][4];
  double time;                /* time stamp for the estimate */
} Estimate;

typedef struct _obs_args {
  Observation obs;
} ObsArgs;

typedef struct _est_args {
  double scale;
  Estimate est;
} EstArgs;

typedef struct _drawing_request {
  int obs_number;
  ObsArgs obs_args[NMAX_OBS];

  int est_number;
  EstArgs est_args[NMAX_EST];

  int draw_history;
  int draw_streamline;

} DrawingRequest;
// ***************************************************

typedef struct Robot_interface {    /* DO NOT ALTER */
  // SENSORS
  double eye_theta[NEYES];
  double eye_theta_dot[NEYES];
  int image[NEYES][NPIXELS][NPRIMARY_COLORS]; /* rgb */
  double arm_theta[NARMS][NARM_JOINTS];
  double arm_theta_dot[NARMS][NARM_JOINTS];
  double ext_force[NARMS][2];       /* (fx,fy) force on arm endpoint */
  double ext_force_body[2];         // (fx,fy) force sensor on roger body
  double base_position[3];          /* x,y,theta */
  double base_velocity[3];
  double wheel_theta_dot[NWHEELS];

  // MOTORS
  double eye_torque[NEYES];
  double arm_torque[NARMS][NARM_JOINTS];
  double wheel_torque[NWHEELS];
  // TELEOPERATOR
  int enter_param_event;
  int button_event;
  double button_reference[2];
  // CONTROL MODE
  int control_mode;
  int input_mode;
  // ROOM
  int room_num;
  Map world_map, arm_map[NARMS];
  // REFERENCE VALUE
  double base_setpoint[3]; /* desired world frame base position (x,y,theta) */
  double arm_setpoint[NARMS][NARM_JOINTS];      /* desired arm joint angles */
  double eyes_setpoint[NEYES];                    /* desired eye pan angle  */
  // TIME
  double simtime;
  // VISUALIZATION RELATED PARAMETERS
  Graphics graphics;
  DrawingRequest drawing_request;
  // ENVIRONMENT (ARENA vs. DEVELOPMENTAL)
  int environment;

} Robot;



typedef struct _vertex
{
  double q[7];
} Vertex;

typedef struct _edge
{
  int v1,v2;
}Edge;

Vertex v[NSTEPS];
Edge e[NSTEPS];
double edge_distance[NSTEPS][NSTEPS];
int next[NSTEPS][NSTEPS];
int num_edges;


typedef struct _pr_dist {
  double bin_size;
  double area;
  double dist[NBINS];
} Pr_dist;

