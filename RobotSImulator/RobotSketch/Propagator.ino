void getdeltapos(float *xdelt, float *ydelt, float *thetadelt){
  float enc1, enc2, rturn;
  getEncoderDelta(&enc1, &enc2);
  *thetadelt = (enc2 - enc1)/(RADIUS1+RADIUS2);
  if(enc1 != enc2){
    rturn = (enc1*RADIUS2+enc2*RADIUS1)/(enc2-enc1);
    *xdelt = -rturn*sin(theta)+rturn*sin(theta+*thetadelt);
    *ydelt = rturn*cos(theta)-rturn*cos(theta+*thetadelt);
  }else{
    *xdelt = enc1*cos(theta);
    *ydelt = enc2*sin(theta);
  }
}
void propagate(float xdelt, float ydelt, float thetadelt) {
  xpos += xdelt;
  ypos += ydelt;
  theta += thetadelt;
  if (theta > 2 * M_PI) theta -= 2 * M_PI;
  if (theta < -2 * M_PI) theta += 2 * M_PI;
}
