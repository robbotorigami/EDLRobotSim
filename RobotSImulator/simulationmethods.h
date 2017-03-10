#pragma once

class Simulation {
public:
	Simulation();
	void setState(double xpos, double ypos, double theta);
	void getState(double *xpos, double *ypos, double *theta);
	void setStateDeriv(double xDot, double yDot, double thetaDot);
	void propagate(double timestep);
private:
	double time;
	double xpos, ypos, theta;
	double xDot, yDot, thetaDot;
};
extern Simulation mainSim;
void commandMotion(float speed, float rotationRate);
void getdeltapos(float* xdelt, float* ydelt, float* thetadelt);