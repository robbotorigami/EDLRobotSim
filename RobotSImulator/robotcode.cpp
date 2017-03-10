#include "dubins.h"
#define _USE_MATH_DEFINES
#include <cmath>

#define SIMULATION
#ifdef SIMULATION
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "simulationmethods.h"
using namespace std;
ofstream modelpathfile;
#endif

#ifdef SIMULATION
#define DEBUG(string) printf(string)
#else
#define DEBUG(string) Serial.println(string)
#endif

#define MAX_SPEED 0.3 //[m/s]

#define MAX_ANGULAR_VEL 3 //[rad/s]

float path[100][2];
size_t pathlength = 0;

enum stage_t{
	STAGE1, //Go from first square to second
	STAGE2, //Rotate 
	STAGE3,
	STAGE4,
}stage;

float xpos;
float ypos;
float theta;
void propagate(float xdelt, float ydelt, float thetadelt);
int findclosestpoint(float xpos, float ypos);
void computeActuation(int index, float xpos, float ypos, float theta,  float* velocity, float* angularvel);

int printConfiguration(double q[3], double x, void* user_data) {
#ifdef SIMULATION
	char buffer[1000];
	ofstream myfile;
	myfile.open("dubinspath.csv", ios::app);
	sprintf_s(buffer, "%f,%f,%f,%f\n", q[0], q[1], q[2], x);
	myfile << buffer;
	myfile.close();
#endif
	if (pathlength >= 98) {
		DEBUG("INVALID PATH\n");
		while (true);
	}
	path[pathlength][0] = q[0];
	path[pathlength][1] = q[1];
	pathlength++;
	return 0;
}

void setup() {
	double q0[] = { 0,0,3.142/2};
	double q1[] = { -2,1,3*3.142/2};
	DubinsPath path;
	dubins_init(q0, q1, 0.8, &path);

#ifdef SIMULATION
	remove("dubinspath.csv");
	remove("robotsmodel.csv");
	modelpathfile.open("robotsmodel.csv");
#endif
	DEBUG("Calculating Dubins path\n");
	dubins_path_sample_many(&path, printConfiguration, 0.1, NULL);
	DEBUG("done\n");

	xpos = q0[0];
	ypos = q0[1];
	theta = q0[2];
	printConfiguration(q1, 0, NULL);
	::path[pathlength][0] = q1[0] + 1 * cos(q1[2]);
	::path[pathlength][1] = q1[0] + 1 * sin(q1[2]);
	stage = STAGE1;
#ifdef SIMULATION
	printf("path length: %d\n", pathlength);
#endif
}

float prev_error = 1E8;

void loop() {
	float xdelt, ydelt, thetadelt;
	getdeltapos(&xdelt, &ydelt, &thetadelt);
	propagate(xdelt, ydelt, thetadelt);
	float speed, angularvel;
	switch (stage) {
	case STAGE1:
		float error = pow(abs(xpos - path[pathlength - 1][0]), 2) + pow(abs(ypos - path[pathlength - 1][1]), 2);
		if (error < 0.3) {
			if (error > prev_error) {
				stage = STAGE2;
				speed = angularvel = 0;
				break;
			}
			prev_error = error;
		}
		int index = findclosestpoint(xpos, ypos);
		computeActuation(index, xpos, ypos, theta, &speed, &angularvel);
		break;
	case STAGE2:

	}

	commandMotion(speed, angularvel);

#ifdef SIMULATION
	modelpathfile << xpos << ", " << ypos << ", " << theta << endl;
#endif
}

void propagate(float xdelt, float ydelt, float thetadelt) {
	xpos += xdelt;
	ypos += ydelt;
	theta += thetadelt;
	if (theta > 2 * M_PI) theta -= 2 * M_PI;
	if (theta < -2 * M_PI) theta += 2 * M_PI;
}


int findclosestpoint(float xpos, float ypos) {
	float bestdist = 1E8;
	int index;
	for (int i = 0; i < pathlength; i++) {
		float dist = pow(path[i][0] - xpos, 2) + pow(path[i][1] - ypos, 2);
		if (dist < bestdist) {
			bestdist = dist;
			index = i;
		}
	}
	return index;
}


void computeActuation(int index, float xpos, float ypos, float theta, float* velocity, float* angularvel) {
	float dx = path[index + 1][0] - path[index][0];
	float dy = path[index + 1][1] - path[index][1];
	float desiredangle = atan2(dy,dx);
	float angleerror = desiredangle - theta;
	if (angleerror < -M_PI) angleerror += 2*M_PI;
	if (angleerror >  M_PI) angleerror -= 2*M_PI;
	float ex = xpos - path[index][0];
	float ey = ypos - path[index][1];
	angleerror += (ex*dy - ey*dx) / (2*MAX_SPEED) / (dx*dx + dy*dy);
	*angularvel = angleerror *MAX_ANGULAR_VEL / (M_PI);
	if (*angularvel > MAX_ANGULAR_VEL) *angularvel = MAX_ANGULAR_VEL;
	if (*angularvel < -MAX_ANGULAR_VEL) *angularvel = -MAX_ANGULAR_VEL;
	*velocity = MAX_SPEED - MAX_SPEED / MAX_ANGULAR_VEL * (*angularvel);
}
