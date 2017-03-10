#include "simulationmethods.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdlib>
#include <stdio.h>
using namespace std;

Simulation mainSim;

Simulation::Simulation() 
	:xpos(0),ypos(0),theta(0),xDot(0),yDot(0),thetaDot(0),time(0)
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

void Simulation::setStateDeriv(double xDot, double yDot, double thetaDot) {
	this->xDot = xDot;
	this->yDot = yDot;
	this->thetaDot = thetaDot;
}

void Simulation::propagate(double timestep) {
	this->xpos += xDot*timestep;
	this->ypos += yDot*timestep;
	this->theta += thetaDot*timestep;
	if (theta > 2 * M_PI) theta -= 2 * M_PI;
	if (theta < 0) theta += 2 * M_PI;
	time += timestep;
}

void commandMotion(float speed, float rotationRate) {
	double xpos, ypos, theta;
	double xDot, yDot, thetaDot;
	mainSim.getState(&xpos, &ypos, &theta);
	xDot = speed * cos(theta) *(((rand() % 100) / 100.0 - 0.5) * 0.1 +1.0);
	yDot = speed * sin(theta) *(((rand() % 100) / 100.0 - 0.5) * 0.1 + 1.0);
	thetaDot = rotationRate *(((rand() % 100) / 100.0 - 0.5) * 0.1 + 1.0);
	mainSim.setStateDeriv(xDot, yDot, thetaDot);
}

void getdeltapos(float* xdelt, float* ydelt, float* thetadelt) {
	static double lastx = 0, lasty = 0, lasttheta = 0;
	static bool firstTime = true;
	double currx, curry, currtheta;
	if (firstTime) {
		mainSim.getState(&lastx, &lasty, &lasttheta);
		firstTime = false;
	}

	mainSim.getState(&currx, &curry, &currtheta);
	*xdelt = currx - lastx + ((rand() % 100) / 100.0 - 0.5) * 0.01;
	*ydelt = curry - lasty + ((rand() % 100) / 100.0 - 0.5) * 0.01;
	*thetadelt = currtheta - lasttheta + ((rand() % 100) / 100.0 - 0.5) * 0.01;
	if (*thetadelt > 2 * M_PI) *thetadelt -= 2 * M_PI;
	if (*thetadelt < -2 * M_PI) *thetadelt += 2 * M_PI;
	lastx = currx;
	lasty = curry;
	lasttheta = currtheta;
}
