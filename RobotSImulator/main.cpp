#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include "robotcode.h"
#include "simulationmethods.h"

#define TIMESTEP 0.001 //[s]
#define SIMLENGTH 20.0 //[s]
#define LOOP_PERIOD 0.20 //[s]

struct simState {
	double t;
	double x;
	double y;
	double theta;
	simState(double t, double x, double y, double theta) :t(t), x(x), y(y), theta(theta) {}
};

std::vector<simState> simVals;

int main() {
	printf("--------------------------------------INITIALIZATION---------------------------------\n\r");
	setup();
	mainSim.setState(0, 0, 3.142 / 2);
	printf("Running Sim");
	double simTimeMarker = SIMLENGTH / 50.0;
	double loopTimeMarker = LOOP_PERIOD;
	for (double t = 0; t < SIMLENGTH; t += TIMESTEP) {
		if (t > loopTimeMarker) {
			loop();
			loopTimeMarker += LOOP_PERIOD;
		}
		mainSim.propagate(TIMESTEP);
		double x, y, theta;
		mainSim.getState(&x, &y, &theta);
		simVals.push_back(simState(t,x,y,theta));
		if (t > simTimeMarker) {
			printf("-");
			simTimeMarker += SIMLENGTH / 50.0;
		}
	}
	printf("\nSim Complete!\n");
	
	remove("robotspath.csv"); 
	std::ofstream myfile;
	myfile.open("robotspath.csv", std::ios::app);
	for (auto state : simVals) {
		myfile << state.t << "," << state.x << "," << state.y << "," << state.theta << std::endl;
	}

	myfile.close();


	system("python makePlots.py");
}