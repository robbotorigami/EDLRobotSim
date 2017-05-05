#ifdef SIMULATION
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "simulationmethods.h"
using namespace std;
ofstream modelpathfile;
#define _USE_MATH_DEFINES
#include <cmath>
#else
#include<math.h>
#include <SoftwareSerial.h>
#include <String.h>
#endif

#ifdef SIMULATION
#define DEBUG(string) printf(string)
#define PRINT(...) printf(__VA_ARGS__)
#define APRINT(...)
#else
#define DEBUG(string) Serial.print(string)
#define PRINT(...) 
#define APRINT(...) Serial.print(__VA_ARGS__)
#endif

//-------------Control Parameters-----------
#define MAX_SPEED         0.5 //[m/s]
#define MAX_ANGULAR_VEL   1.0 //[rad/s]
#define MINIMUM_RAD_CURV  0.3 //[m]
#define POINT1_X          0.0 //[m]
#define POINT1_Y          0.0 //[m]
#define POINT1_THETA      (3.142/2.0) //[rad]
#define POINT2_X          0.2 //[m]
#define POINT2_Y          0.5 //[m]
#define POINT2_THETA      (3.142/2.0) //[rad]
#define D_WHEEL           0.13335
const float RADIUS1=      0.2794/2;//[m]
const float RADIUS2=      0.2794/2;//[m]

//---------------Path Storage---------------
float path[100][2];
size_t pathlength = 0;

//---------------Robot State----------------
enum stage_t{
  STAGE1, //Go from first square to second
  STAGE2, //Rotate 360
  STAGE3, //Go from second square to first
  STAGE4,
}stage;

float xpos;
float ypos;
float theta;

//===================EXTERNAL FUNCTIONS=========================
//------------------Propagator Functions------------------------
void getdeltapos(float *xdelt, float *ydelt, float *thetadelt);
void propagate(float xdelt, float ydelt, float thetadelt);
//-----------------Path Planning Functions----------------------
void generatePath(double q1[3], double q2[3]);
int findclosestpoint(float xpos, float ypos);
int addNodeToPath(double q[3], double x, void* user_data);
//----------------Forward Control Functions---------------------
void computeActuation(int index, float xpos, float ypos, float theta,  float* velocity, float* angularvel);
void commandMotion(float cmdSpeed, float cmdAngularVel);
//--------------Hardware Abstraction Functions------------------
void HALInit();
void commandLeftWheel(float cmdSpeed);
void commandRightWheel(float cmdSpeed);
void getEncoderDelta(float *leftEnc, float *rightEnc);
//=================END EXTERNAL FUNCTIONS=======================

#define pathPoint(node, x, y) do{node[0] = x; node[1] = y; addNodeToPath(node,0.0,0);}while(false)

void setup() {
#ifdef SIMULATION
  remove("dubinspath.csv");
  remove("robotsmodel.csv");
  modelpathfile.open("robotsmodel.csv");
  ofstream myfile;
  myfile.open("dubinspath.csv", ios::app);
  myfile.close();
#endif
  double q0[] = {POINT1_X, POINT1_Y, POINT1_THETA};
  double q1[] = {POINT2_X, POINT2_Y, POINT2_THETA};


  generatePath(q0, q1);
  xpos = q0[0];
  ypos = q0[1];
  theta = q0[2];
  stage = STAGE1;
  /*
  double node[3] = {0};
  pathPoint(node, 0, 0.0);
  pathPoint(node, 0, 0.1);
  pathPoint(node, 0, 0.2);
  pathPoint(node, 0, 0.3);
  pathPoint(node, 0, 0.4);
  path[pathlength][0] = 0.0;
  path[pathlength][1] = 0.45;
  xpos = 0;
  ypos = 0;
  theta = M_PI/2;*/
  HALInit();
}

#ifdef SIMULATION
void loop() {
  float xdelt, ydelt, thetadelt;
  getdeltapos(&xdelt, &ydelt, &thetadelt);
  propagate(xdelt, ydelt, thetadelt);
  APRINT("STAGE: "); APRINT(stage); APRINT("  "); APRINT("Position: "); APRINT(xpos); APRINT(", "); APRINT(ypos); APRINT(", "); APRINT(theta); APRINT("\n");
  static float prev_error = 1E8;
  float cmdSpeed = 0, cmdAngularvel = 0;
  int index;
  float error;
  switch (stage) {
  case STAGE1:
	  index = findclosestpoint(xpos, ypos);
	  computeActuation(index, xpos, ypos, theta, &cmdSpeed, &cmdAngularvel);
	  error = pow(abs(xpos - path[pathlength - 1][0]), 2) + pow(abs(ypos - path[pathlength - 1][1]), 2);
	  if (error < pow(0.3, 2)) {
		  cmdSpeed = (3 * pow(error, 1 / 2.0))*(cmdSpeed);
		  cmdAngularvel = (3 * pow(error, 1 / 2.0))*(cmdAngularvel);
		  if (prev_error < error) {
			  cmdSpeed = cmdAngularvel = 0;
			  stage = STAGE2;
		  }
		  prev_error = error;
	  }
	  break;
  default:
	  ;
  }

  commandMotion(cmdSpeed, cmdAngularvel);

#ifdef SIMULATION
  modelpathfile << xpos << ", " << ypos << ", " << theta << endl;
#endif
}

#else
struct state_t{
  float x;
  float y;
  float theta;
};

extern void read_new_state();
extern struct state_t state;

extern SoftwareSerial softSer;
void loop(){
  read_new_state();
  APRINT("Commanded to: "); APRINT(state.x); APRINT(", "); APRINT(state.y); APRINT(", "); APRINT(state.theta); APRINT("\n");
  double q0[] = {POINT1_X, POINT1_Y, POINT1_THETA};
  double q1[] = {state.x, state.y, state.theta};
  generatePath(q0, q1);
  xpos = q0[0];
  ypos = q0[1];
  theta = q0[2];
  float prev_error = 1E8;
  unsigned long timeout = millis();
  float cmdSpeed = 0, cmdAngularvel = 0;
  while(true){
    float xdelt, ydelt, thetadelt;
    getdeltapos(&xdelt, &ydelt, &thetadelt);
    propagate(xdelt, ydelt, thetadelt);
    
    int index;
    float error;
          
    index = findclosestpoint(xpos, ypos);
    computeActuation(index, xpos, ypos, theta, &cmdSpeed, &cmdAngularvel);
    error = pow(pow(fabs(xpos - path[pathlength - 1][0]), 2) + pow(fabs(ypos - path[pathlength - 1][1]), 2), 1.0/2.0) 
      + 0.005*fabs(theta - path[pathlength -1][2]);
    if (error < 0.3) {
      APRINT("RAMPING  ");
      cmdSpeed = (2 * error + 0.3)*(cmdSpeed);
      cmdAngularvel = (2 * error + 0.3)*(cmdAngularvel);
      if (prev_error < error) {
        break;
      }
      prev_error = error;
    }    
    if(millis() - timeout > 20000) break;  
    APRINT("ERROR: "); APRINT(error); APRINT(", ");           
   APRINT("Position: "); APRINT(xpos); APRINT(", "); APRINT(ypos); APRINT(", "); APRINT(theta); APRINT("\n");
  
    commandMotion(cmdSpeed, cmdAngularvel);
  }
  
   cmdSpeed = cmdAngularvel = 0;
   commandMotion(cmdSpeed, cmdAngularvel);
  
}
#endif


