/**************************************************************************/
/* File:        simulate.h                                                */
/* Description: all the compile time constants for use in the simulator   */
/* Author:      Rod Grupen                                                */
/* Date:        11-1-2009                                                 */
/**************************************************************************/
#define VERBOSE          0 // switches a whole bunch of printf milestones
#define COUPLED_DYNAMICS 1 // 1 - whole body, 0 - block diagonal
/********************* simulator X display **********************/

// simulator panel
//#define WIDTH              1260//1200//1060
//#define HEIGHT             480
#define WIDTH              1260//1200//1060
#define HEIGHT             560
#define WIDTH_DEV          1060
#define HEIGHT_DEV         560
#define PIX_MAP_SIZE_RATIO  2.5 // 1.5

#define ARENA              0
#define DEVELOPMENTAL      1


//cartesian space
#define MIN_X              -5.0//-4.5 //-2.0
#define MIN_Y              -2.0
#define MAX_X              5.0//4.5 //2.0
#define MAX_Y              2.0
#define MIN_X_DEV          -2.0
#define MIN_Y_DEV          -2.0
#define MAX_X_DEV           2.0
#define MAX_Y_DEV           2.0
#define NBINS              64
#define XDELTA       ((MAX_X-MIN_X)/NBINS)
#define YDELTA       ((MAX_Y-MIN_Y)/NBINS)
#define XDELTA_DEV       ((MAX_X_DEV-MIN_X_DEV)/NBINS)
#define YDELTA_DEV       ((MAX_Y_DEV-MIN_Y_DEV)/NBINS)
#define R_OBSTACLE         (XDELTA/2.0)
#define R_OBSTACLE_DEV     (XDELTA_DEV/2.0)


#define T1                0
#define T2                1
#define T1_MIN      -(M_PI)
#define T2_MIN      -(M_PI)
#define T1_MAX       (M_PI)
#define T2_MAX       (M_PI)
#define TDELTA   ((2.0*M_PI)/NBINS)

// button geometry
#define BOXH             18
#define BOXW             ((int) WIDTH/4 - 4) //five buttons will fit
#define BOXW_DEV         ((int) WIDTH_DEV/7 - 7) //seven buttons will fit

// sub-panel geometry and placement
#define LEFT_MAP_CENTER_X  170
#define LEFT_MAP_CENTER_Y  220
#define CENTER_X           600 //530
#define CENTER_X_DEV       530
#define CENTER_Y           220
#define RIGHT_MAP_CENTER_X 890
#define RIGHT_MAP_CENTER_Y 220

#define ZOOM_SCALE     (100.0)
#define MAP_SCALE          (50.0)
/* world to pixmap panels (drawing) transformations: Arena */
#define W2DR(scale,num)          (((int)(scale*ZOOM_SCALE*1000.0*(num))/1000))
#define W2DX(scale,num)    ((int)(scale*CENTER_X)+((int)(scale*ZOOM_SCALE*1000.0*((num)))/1000))
#define W2DY(scale,num)    ((int)(scale*CENTER_Y)-((int)(scale*ZOOM_SCALE*1000.0*((num)))/1000))
/* pixmap to world: Arena */
#define D2WR(scale,num)    ((double)(num*1000)/(scale*ZOOM_SCALE*1000.0))
#define D2WX(scale,num)    ((double)((num-scale*CENTER_X)*1000)/(scale*ZOOM_SCALE*1000.0))
#define D2WY(scale,num)    ((double)((scale*CENTER_Y-num)*1000)/(scale*ZOOM_SCALE*1000.0))

/* world to pixmap panels (drawing) transformations: Developmental environment */
#define W2DR_DEV(scale,num)          (((int)(scale*ZOOM_SCALE*1000.0*(num))/1000))
#define W2DX_DEV(scale,num)    ((int)(scale*CENTER_X_DEV)+((int)(scale*ZOOM_SCALE*1000.0*((num)))/1000))
#define W2DY_DEV(scale,num)    ((int)(scale*CENTER_Y)-((int)(scale*ZOOM_SCALE*1000.0*((num)))/1000))
/* pixmap to world: Developmental Environment */
#define D2WR_DEV(scale,num)    ((double)(num*1000)/(scale*ZOOM_SCALE*1000.0))
#define D2WX_DEV(scale,num)    ((double)((num-scale*CENTER_X_DEV)*1000)/(scale*ZOOM_SCALE*1000.0))
#define D2WY_DEV(scale,num)    ((double)((scale*CENTER_Y-num)*1000)/(scale*ZOOM_SCALE*1000.0))


// world to display theta
#define T2DR(scale,num)   (((int)(scale*MAP_SCALE*1000.0*(num))/1000))
#define T12LD(scale,num)  (scale*LEFT_MAP_CENTER_X+((int)(scale*MAP_SCALE*1000.0*((num)))/1000))
#define T22LD(scale,num)  (scale*LEFT_MAP_CENTER_Y-((int)(scale*MAP_SCALE*1000.0*((num)))/1000))
#define T12RD(scale,num)  (scale*RIGHT_MAP_CENTER_X+((int)(scale*MAP_SCALE*1000.0*((num)))/1000))
#define T22RD(scale,num)  (scale*RIGHT_MAP_CENTER_Y-((int)(scale*MAP_SCALE*1000.0*((num)))/1000))

#define LEFT_IMAGE_X       (CENTER_X_DEV - (IMAGE_WIDTH+5))
#define RIGHT_IMAGE_X      (CENTER_X_DEV + 5)
#define LEFT_IMAGE_X_1       (CENTER_X - (IMAGE_WIDTH+5)) - 300
#define RIGHT_IMAGE_X_1      LEFT_IMAGE_X_1 + 280 //(CENTER_X + 5)
#define LEFT_IMAGE_X_2       (CENTER_X - (IMAGE_WIDTH+5)) + 300
#define RIGHT_IMAGE_X_2      LEFT_IMAGE_X_2 + 280 //(CENTER_X + 5)

#define IMAGE_Y            440
#define PIXEL_WIDTH        (IMAGE_WIDTH/NPIXELS)
#define PIXEL_HEIGHT       20

/********************* simulation constants **********************/
#define TIMEOUT            60        /* seconds worth of simulation       */
#define DT                 0.001     /* the time increment between frames */
#define RENDER_RATE        1        /* render every twentieth state      */
#define SERVO_RATE         1         /* servo rate at 1000Hz (1 msec)     */
#define TIMER_UPDATE       5
#define IMAGE_RATE       5

#define NOFILL             0
#define FILL               1

//#define RED          -3
//#define GREEN        -2
//#define BLUE         -1

// indices into world_color array (xrobot.c x_init_colors() )
#define DARKRED      101
#define RED          102
#define LIGHTRED     103
#define DARKBLUE     104
#define BLUE         105
#define LIGHTBLUE    106
#define DARKGREEN    107
#define GREEN        108
#define LIGHTGREEN   109
#define DARKYELLOW   110
#define YELLOW       111
#define LIGHTYELLOW  112

#define GAZE_COLOR   65
#define OBJECT_COLOR RED
#define ARM_COLOR    BLUE
#define EYE_COLOR    DARKGREEN
#define GOAL_COLOR   GREEN 

/***********************************************************************/
/* CONSTANTS AND STRUCTURES THAT DEFINE MECHANISMS FOR THE SIMULATOR   */
#define NOTYPE       -1              /* KINEMATIC SPECIFICATIONS       */
#define NOAXIS       -1
#define REVOLUTE      0
#define PRISMATIC     1
#define XAXIS         0
#define YAXIS         1
#define ZAXIS         2

#define MaxRobotNum        3
#define MaxToyNum          1
// 3 objects per robot (base and 2 arms) plus the toys
#define MaxInertialObjectsNum  MaxRobotNum * 3 + MaxToyNum 





/***********************************************************************/
// STRUCTURES FOR THE STATE OF ALL THE DEVICES THAT COMPRISE ROGER
typedef struct _base {
  double wTb[4][4];
  double x;
  double x_dot;
  double y;
  double y_dot;
  double theta;
  double theta_dot;
  double wheel_torque[2];
  double contact_theta;
  double extForce[2];          // net (fx,fy) force on the base
  double wheel_theta_dot[NWHEELS];
} Base;

typedef struct _arm {
  double iTj[4][4];
  int dof_type;                // revolute or prismatic type
  int axis;                    // XAXIS, YAXIS, ZAXIS
  double theta;
  double theta_dot;
  double torque;
  double extForce[2];          // (fx,fy) force on distal endpoint of this link
} Arm;

typedef struct _eye {
  double position[2];          // position of the eye in world coordinates
  double theta;                // eye pan angle relative to world frame
  double theta_dot;            // angular velocity
  int    image[NPIXELS];       // afferent ONLY
  double torque;               // efferent ONLY - command torque
} Eye;

typedef struct _object {
  double mass;                 // intrinsic parameters 
  double position[3];          // position of the centroid of the object
  double velocity[3];          // velocity of the centroid of the object
  double extForce[3];          // written by the simulator: endpoint load
} Obj;

// a composite of N elastic spheres at the vertices of a regular polygon
typedef struct _polyball {
  int id;                // so far CIRCLE || TRIANGLE but any N can be used
  int N;                 // a composite of N elastic spheres at the vertices
  double Rsphere;        // the radius of the elastic spheres
  double radius;         // the distance from the body frame to the center
               // of the elastic sphere
  double mass;           // toal mass of the entire polyball
  double moi;            // moment of inertia ( total_mass * SQR(radius) )
  double position[3];    // position (x,y,theta) of the object
  double velocity[3];    // velocity of the object
  double net_extForce[3]; // from collisions with other objects
} PolyBall;

// A roger body struct which is composed of base, eyes, and arms. This is used
// on the simulator side and for simulating the dynamics
typedef struct _roger_body {
  Base mobile_base;
  Eye eyes[NEYES];
  Arm arms[NARMS][NARM_FRAMES];
} RogerBody;

/*************************************************************************/
//typedef struct _world {
//  int occupancy_map[NBINS][NBINS]; // ground truth world geometry
//  int color_map[NBINS][NBINS];     // index into SimColors for "light" colors
//} World;
/*************************************************************************/

typedef struct _simcolor {
  char name[32];
  int display_color;
  int red;
  int green;
  int blue;
} SimColor;

// eObject (elastic object) data type:
//       N=1: CIRCLE (special case R=Rball)
//   2<N<=12: POLYGON w/N vertices 
//       N=2: default "illegal object"
typedef struct _eobject {
  int N;
  double vertices[12][2];  // 1D array - up to twelve vertices
  SimColor color[12];      // color[0] describes edge 0-1
  double mass;            // total mass
  double moi;             // moment of inertia (computed from list of vertices)
  double mu;              // how should we handle? it's a two-body property
  double pos[3];          // position (x,y,theta) of the object COM
  double vel[3];          // velocity of the object
  double net_Fext[3];     // collisions with other objects in object frame
} eObject;

typedef struct _wheel {
  int leftTicks;
  int rightTicks;
  double time;
} Wheel;


// Toy Box space
#define TOY_MIN_X         -2.875
#define TOY_MAX_X          2.875
#define TOY_MIN_Y         -3.5
#define TOY_MAX_Y         -2.5

#define N_TOY_TYPES    5   // the number of types of toys
#define MAX_TOYS      10   // the maximum number of active toys allowed

/*************************************************************************/

#define MAX_HISTORY   1000

typedef struct _history {
  double arm_pos[NARMS][2];    /* (theta1, theta2) */
  double base_pos[3];          /* (x,y,theta) */
} History;


void project1_enter_params(), project1_reset(),
project1_visualize(), project1_control();
void project2_enter_params(), project2_reset(),
project2_visualize(), project2_control();
void project3_enter_params(), project3_reset(),
project3_visualize(), project3_control();
void project4_enter_params(), project4_reset(),
project4_visualize(), project4_control();
void project5_enter_params(), project5_reset(),
project5_visualize(), project5_control();
void project6_enter_params(), project6_reset(),
project6_visualize(), project6_control();
void project7_enter_params(), project7_reset(),
project7_visualize(), project7_control();
void project8_enter_params(), project8_reset(),
project8_visualize(), project8_control();
void project9_enter_params(), project9_reset(),
project9_visualize(), project9_control();
void project10_enter_params(), project10_reset(),
project10_visualize(), project10_control();
void project11_enter_params(), project11_reset(),
project11_visualize(), project11_control();

