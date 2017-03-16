#define SIMULATION

#define _USE_MATH_DEFINES
#include <cmath>

#include "RobotSketch/RobotSketch.ino"
#include "RobotSketch/PathPlanning.ino"
#include "RobotSketch/ForwardControl.ino"
#include "RobotSketch/Propagator.ino"

//-------------------------------SIM HAL----------------------------------
void HALInit() {
	//Do Nothing
}

//cmdSpeed - [m/s] speed of left wheel
void commandLeftWheel(float cmdSpeed) {
	mainSim.setLeftMotor(cmdSpeed);
}

//cmdSpeed - [m/s] speed of right wheel
void commandRightWheel(float cmdSpeed) {
	mainSim.setRightMotor(cmdSpeed);
}

//leftEnc - [m] accumulated distance traveled by left wheel
//rightEnc - [m] accumulated distance traveled by right wheel
void getEncoderDelta(float *leftEnc, float *rightEnc) {
	double left, right;
	mainSim.getEncoderDeltas(&left, &right);
	*leftEnc = (float)left;
	*rightEnc = (float)right;
}
