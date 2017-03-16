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
#endif

#ifdef SIMULATION
#define DEBUG(string) printf(string)
#define PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG(string) Serial.println(string)
#define PRINT(...) 
#endif

//-------------Control Parameters-----------
#define MAX_SPEED         0.3 //[m/s]
#define MAX_ANGULAR_VEL   0.5 //[rad/s]
#define MINIMUM_RAD_CURV  0.3 //[m]
#define POINT1_X          0.0 //[m]
#define POINT1_Y          0.0 //[m]
#define POINT1_THETA      (3.142/2.0) //[rad]
#define POINT2_X         -0.75 //[m]
#define POINT2_Y          0.5 //[m]
#define POINT2_THETA      (3.0*3.142/2.0) //[rad]
const float RADIUS1=      0.1;//[m]
const float RADIUS2=      0.1;//[m]

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
//----------------Forward Control Functions---------------------
void computeActuation(int index, float xpos, float ypos, float theta,  float* velocity, float* angularvel);
void commandMotion(float cmdSpeed, float cmdAngularVel);
//--------------Hardware Abstraction Functions------------------
void HALInit();
void commandLeftWheel(float cmdSpeed);
void commandRightWheel(float cmdSpeed);
void getEncoderDelta(float *leftEnc, float *rightEnc);
//=================END EXTERNAL FUNCTIONS=======================

void setup() {
  double q0[] = {POINT1_X, POINT1_Y, POINT1_THETA};
  double q1[] = {POINT2_X, POINT2_Y, POINT2_THETA};

#ifdef SIMULATION
  remove("dubinspath.csv");
  remove("robotsmodel.csv");
  modelpathfile.open("robotsmodel.csv");
#endif
  generatePath(q0, q1);
  xpos = q0[0];
  ypos = q0[1];
  theta = q0[2];
  stage = STAGE1;
  HALInit();
}

void stage1Handler(float *cmdSpeed, float *cmdAngularvel){
    static float prev_error = 1E8;
    int index = findclosestpoint(xpos, ypos);
    computeActuation(index, xpos, ypos, theta, cmdSpeed, cmdAngularvel);
    float error = pow(abs(xpos - path[pathlength-1][0]), 2) + pow(abs(ypos - path[pathlength-1][1]), 2);
    if (error < 0.3) {
      if (error > prev_error) {
        double q0[] = {POINT1_X, POINT1_Y, POINT1_THETA};
        double q1[] = {POINT2_X, POINT2_Y, POINT2_THETA};
        stage = STAGE2;
        *cmdSpeed = *cmdAngularvel = 0;
        commandMotion(*cmdSpeed, *cmdAngularvel);
        generatePath(q1, q0); //Computing the path here will give us a delay
        return;
      }else{
        *cmdSpeed = (3*error+0.05)*(*cmdSpeed);
        *cmdAngularvel = (3*error+0.05)*(*cmdAngularvel);
      }
      prev_error = error;
    }
}
void stage2Handler(float *cmdSpeed, float *cmdAngularvel){
  static float prevtheta = 0;
  static bool passedZero = false;
  if(!passedZero){
    *cmdAngularvel = MAX_ANGULAR_VEL;
    *cmdSpeed = 0;
    if(theta < prevtheta) passedZero = true;
  }else{
    if(theta > 3.0*3.1415/2.0){
      stage = STAGE3;
      *cmdSpeed = *cmdAngularvel = 0;    
      return;      
    }
    float error = (3.0*3.1415/2.0 - theta);
    *cmdSpeed = 0;
    *cmdAngularvel = (0.15*error+0.05)*(MAX_ANGULAR_VEL);
  }
  prevtheta = theta;
  
}
void stage3Handler(float *cmdSpeed, float *cmdAngularvel){
  static float prev_error = 1E8;
  int index = findclosestpoint(xpos, ypos);
  computeActuation(index, xpos, ypos, theta, cmdSpeed, cmdAngularvel);
  float error = pow(abs(xpos - path[pathlength-1][0]), 2) + pow(abs(ypos - path[pathlength-1][1]), 2);
  if (error < 0.3) {
    if (error > prev_error) {
      stage = STAGE4;
      *cmdSpeed = *cmdAngularvel = 0;
      return;
    }else{
      *cmdSpeed = (3*error+0.05)*(*cmdSpeed);
      *cmdAngularvel = (3*error+0.05)*(*cmdAngularvel);
    }
    prev_error = error;
  } 
}
void stage4Handler(float *cmdSpeed, float *cmdAngularvel){
  *cmdSpeed = *cmdAngularvel = 0;
}


void loop() {
  float xdelt, ydelt, thetadelt;
  getdeltapos(&xdelt, &ydelt, &thetadelt);
  propagate(xdelt, ydelt, thetadelt);
  float cmdSpeed, cmdAngularvel;
  switch (stage) {
  case STAGE1:
    stage1Handler(&cmdSpeed, &cmdAngularvel);
    break;
  case STAGE2:
    stage2Handler(&cmdSpeed, &cmdAngularvel);
    break;
  case STAGE3:
    stage3Handler(&cmdSpeed, &cmdAngularvel);
    break;
  case STAGE4:
    stage4Handler(&cmdSpeed, &cmdAngularvel);
    break;
  }

  commandMotion(cmdSpeed, cmdAngularvel);

#ifdef SIMULATION
  modelpathfile << xpos << ", " << ypos << ", " << theta << endl;
#endif
}



