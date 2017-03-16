#include "simulationmethods.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdlib>
#include <stdio.h>
using namespace std;
#define frand(num, frac) ((2*(rand()%num)-num)/((float)num)*frac)

Simulation mainSim;

Simulation::Simulation() 
	:xpos(0),ypos(0),theta(0),time(0),
	rmspeed(0),lmspeed(0),encoderStep1(0),encoderStep2(0)
{
	srand(12341234);
}

void Simulation::setState(double xpos, double ypos, double theta){
	this->xpos = xpos;
	this->ypos = ypos;
	this->theta = theta;
}

void Simulation::getState(double *xpos, double *ypos, double *theta) {
	*xpos = this->xpos;
	*ypos = this->ypos;
	*theta = this->theta;
}

void Simulation::propagate(double timestep) {
	double thetaStep = 0, xStep = 0, yStep = 0;
	thetaStep = timestep * (rmspeed - lmspeed) / (RADIUS1 + RADIUS2);
	if (rmspeed != lmspeed) {
		double rturn = (lmspeed*RADIUS2 + rmspeed*RADIUS1) / (rmspeed - lmspeed);
		xStep = -rturn*sin(theta) + rturn*sin(theta + thetaStep);
		yStep = rturn*cos(theta) - rturn*cos(theta + thetaStep);
	}
	else {
		xStep = timestep*rmspeed*cos(theta);
		yStep = timestep*rmspeed*sin(theta);
	}
	this->xpos += xStep;
	this->ypos += yStep;
	this->theta += thetaStep;
	if (theta > 2 * M_PI) theta -= 2 * M_PI;
	if (theta < 0) theta += 2 * M_PI;
	time += timestep;
	encoderStep1 += timestep*lmspeed;
	encoderStep2 += timestep*rmspeed;
}

void Simulation::setLeftMotor(double leftMotor) {
	lmspeed = leftMotor*(1.0+frand(100,0.1));
}
void Simulation::setRightMotor(double rightMotor) {
	rmspeed = rightMotor*(1.0 + frand(100, 0.1));
}
void Simulation::getEncoderDeltas(double* enc1, double* enc2) {
	*enc1 = encoderStep1*(1.0 + frand(100, 0.05));
	*enc2 = encoderStep2*(1.0 + frand(100, 0.05));
	encoderStep1 = 0;
	encoderStep2 = 0;
}