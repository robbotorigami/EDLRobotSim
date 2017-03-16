void computeActuation(int index, float xpos, float ypos, float theta, float* velocity, float* angularvel) {
  float dx = path[index + 1][0] - path[index][0];
  float dy = path[index + 1][1] - path[index][1];
  float desiredangle = atan2(dy,dx);
  float angleerror = (desiredangle - theta);
  if (angleerror < -M_PI) angleerror += 2*M_PI;
  if (angleerror >  M_PI) angleerror -= 2*M_PI;
  angleerror *= 3.0;
  float ex = xpos - path[index][0];
  float ey = ypos - path[index][1];
  angleerror += 2.0*(ex*dy - ey*dx) / (2*MAX_SPEED) / (dx*dx + dy*dy);
  *angularvel = angleerror *MAX_ANGULAR_VEL / (M_PI);
  if (*angularvel > MAX_ANGULAR_VEL) *angularvel = MAX_ANGULAR_VEL;
  if (*angularvel < -MAX_ANGULAR_VEL) *angularvel = -MAX_ANGULAR_VEL;
  *velocity = MAX_SPEED - MAX_SPEED / MAX_ANGULAR_VEL * (*angularvel);
}

void commandMotion(float cmdSpeed, float cmdAngularVel){
  commandLeftWheel(cmdSpeed - (MAX_SPEED/MAX_ANGULAR_VEL) * cmdAngularVel);
  commandRightWheel(cmdSpeed + (MAX_SPEED/MAX_ANGULAR_VEL) * cmdAngularVel);
}

