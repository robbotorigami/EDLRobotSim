class PIDController {
public:
	PIDController(float kp, float ki, float kd, float intmax) 
		:kp(kp), ki(ki), kd(kd), intmax(intmax), accum(0), preverror(0) {}
	float calculate(float error) {
		float prop, integ, deriv;

		prop = kp * error;

		accum += error;
		if (accum > intmax) accum = intmax;
		if (accum < -intmax) accum = -intmax;
		integ = ki*accum;

		deriv = kd*(error - preverror);
		preverror = error;

		return prop + integ + deriv;
	}
private:
	//Configuration
	float kp;
	float ki;
	float kd;
	float intmax;

	//Integral/derivative computation
	float accum;
	float preverror;
};

PIDController pid(6.0, 0.5, 0.1, 2.0);

void computeActuation(int index, float xpos, float ypos, float theta, float* velocity, float* angularvel) {
  float error = pow(pow(fabs(xpos - path[pathlength - 1][0]), 2) + pow(fabs(ypos - path[pathlength - 1][1]), 2), 1.0/2.0) ;
  float dx = path[index + 1][0] - path[index][0];
  float dy = path[index + 1][1] - path[index][1];
  float desiredangle = atan2(dy,dx);
  float angleerror = (desiredangle - theta);
  if (angleerror < -M_PI) angleerror += 2*M_PI;
  if (angleerror >  M_PI) angleerror -= 2*M_PI;
  angleerror /= M_PI;
  float ex = xpos - path[index][0];
  float ey = ypos - path[index][1];
  float torque = (ex*dy - ey*dx) * 50;
  if (torque > M_PI) torque = M_PI;
  if (torque <-M_PI) torque =-M_PI;
  if(error > 0.3){
    angleerror += 2*torque; //trying this out    
  }else{
    angleerror += 4*torque;
  }
  angleerror += pid.calculate(angleerror);
  if (fabs(angleerror) > 1) angleerror /= fabs(angleerror); //limit angle error to -1 to 1
  *angularvel = angleerror *MAX_ANGULAR_VEL;
  *velocity = MAX_SPEED - MAX_SPEED / MAX_ANGULAR_VEL * fabs(*angularvel);
}

void commandMotion(float cmdSpeed, float cmdAngularVel){
  commandLeftWheel(cmdSpeed - (MAX_SPEED/MAX_ANGULAR_VEL) * cmdAngularVel);
  commandRightWheel(cmdSpeed + (MAX_SPEED/MAX_ANGULAR_VEL) * cmdAngularVel);
}

