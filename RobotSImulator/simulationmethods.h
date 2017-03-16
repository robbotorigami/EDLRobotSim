#pragma once

class Simulation {
public:
	Simulation();
	void setState(double xpos, double ypos, double theta);
	void getState(double *xpos, double *ypos, double *theta);
	void setLeftMotor(double leftMotor);
	void setRightMotor(double rightMotor);
	void getEncoderDeltas(double* enc1, double* enc2);
	void propagate(double timestep);
private:
	double time;
	double lmspeed, rmspeed;
	double encoderStep1, encoderStep2;
	double xpos, ypos, theta;
};
extern Simulation mainSim;
extern const float RADIUS1;
extern const float RADIUS2;