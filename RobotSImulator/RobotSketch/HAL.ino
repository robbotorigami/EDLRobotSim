
#define pinEnc_Left_Enc1   3
#define pinEnc_Left_Enc2   10
#define pinEnc_Right_Enc1  2
#define pinEnc_Right_Enc2  11
#define pinON              6
#define pinCW_Left         9    
#define pinCC_Left         8   
#define pinSpeed_Left      6    
#define pinCW_Right       7  
#define pinCC_Right       4
#define pinSpeed_Right    5

#define HC06_RX                 12
#define HC06_TX                 13

//THERE ARE TWO ENCODER LINES PER ENCODER
volatile signed long enc_right_pulses = 0, enc_left_pulses = 0;

void EncRight_IRQHandler(){

  if(digitalRead(pinEnc_Right_Enc2) == HIGH)
    enc_right_pulses++;
  else
    enc_right_pulses--;
}

void EncLeft_IRQHandler(){
  
  if(digitalRead(pinEnc_Left_Enc2) == HIGH)
    enc_left_pulses--;
  else
    enc_left_pulses++;
}


struct state_t state;
SoftwareSerial softSer(HC06_RX, HC06_TX); // RX, TX

void read_new_state_var(float* state_var, char delimiter){
  String buf = "";
  while(!softSer.available());
  char new_char = softSer.read();
  while(new_char != delimiter){
   buf += new_char;
   do{ new_char = softSer.read(); }while(new_char == -1);
  }

  *state_var = buf.toFloat();
#ifdef DEBUG
  Serial.print("string read: ");
  Serial.println(buf);
  Serial.print("string.toFloat(): ");
  Serial.println(*state_var);
#endif
}

void read_new_state(){
  
  char xy_delim = ',';
  char theta_delim = '\n';
  do{
    read_new_state_var(&(state.x), xy_delim);
    read_new_state_var(&(state.y), xy_delim);
    read_new_state_var(&(state.theta), theta_delim);
  }while(false);
}


void HALInit(){
  
  pinMode(pinEnc_Left_Enc1, INPUT_PULLUP);
  pinMode(pinEnc_Right_Enc1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinEnc_Left_Enc1), EncLeft_IRQHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEnc_Right_Enc1), EncRight_IRQHandler, RISING);
  Serial.begin(9600);
  pinMode(pinON,INPUT);
  pinMode(pinEnc_Right_Enc2, INPUT);
  pinMode(pinEnc_Left_Enc2, INPUT);
  pinMode(pinCW_Left,OUTPUT);
  pinMode(pinCC_Left,OUTPUT);
  pinMode(pinSpeed_Left,OUTPUT);
  pinMode(pinCW_Right,OUTPUT);
  pinMode(pinCC_Right,OUTPUT);
  pinMode(pinSpeed_Right,OUTPUT);
  pinMode(13,OUTPUT);
  
  digitalWrite(pinCW_Left,LOW);
  digitalWrite(pinCC_Left,LOW);           
  digitalWrite(pinCW_Right,LOW);          
  digitalWrite(pinCC_Right,LOW);

  softSer.begin(9600);
  
 // while(digitalRead(pinON) == LOW);      // wait for ON switch  digitalWrite(13,LOW);                     // turn LED off
 // delay(1000);
}

#define LIM_CMD_R 85
#define LIM_CMD_L 85

float prev_right = 0;
float prev_left = 0;

//cmdSpeed - [m/s] speed of right wheel
// max cmdSpeed for right wheel ~ 0.52 m/s
void commandRightWheel(float cmdSpeed){
  cmdSpeed /= MAX_SPEED;
  cmdSpeed *= LIM_CMD_R;
  if(cmdSpeed > LIM_CMD_R) cmdSpeed = LIM_CMD_R;
  if(cmdSpeed < -LIM_CMD_R) cmdSpeed = -LIM_CMD_R;
  if(fabs(cmdSpeed) < 10){
      digitalWrite(pinCC_Right, LOW);
      digitalWrite(pinCW_Right, LOW);
      analogWrite(pinSpeed_Right, 0);
      return;
  }
//  if(cmdSpeed > prev_right + (LIM_CMD_R /4.0)) cmdSpeed = prev_right + (LIM_CMD_R/4.0);
//  if(cmdSpeed < prev_right - (LIM_CMD_R /4.0)) cmdSpeed = prev_right - (LIM_CMD_R/4.0);
//  prev_right = cmdSpeed;
  if(cmdSpeed > 0){ // move right wheel cw
      digitalWrite(pinCC_Right, HIGH);
      digitalWrite(pinCW_Right, LOW);
      analogWrite(pinSpeed_Right, fabs(cmdSpeed)); 
  }  else{ // move right wheel ccw
      digitalWrite(pinCC_Right, LOW);
      digitalWrite(pinCW_Right, HIGH);
      analogWrite(pinSpeed_Right, fabs(cmdSpeed)); 
  }
//  cmdSpeed *= 0.8;
//  int awrite = (int)((fabs(cmdSpeed) - 0.0629)/0.0023);
  Serial.print("SpeedR:"); Serial.print(cmdSpeed); 
//  
//  if(cmdSpeed <= 0.0629 && cmdSpeed >= -0.0629){
//      digitalWrite(pinCC_Right, LOW);
//      digitalWrite(pinCW_Right, LOW);
//      analogWrite(pinSpeed_Right, 0);
//  }
//  else if(cmdSpeed > 0){ // move right wheel cw
//      digitalWrite(pinCC_Right, HIGH);
//      digitalWrite(pinCW_Right, LOW);
//      analogWrite(pinSpeed_Right, awrite); 
//  }
//  else{ // move right wheel ccw
//      digitalWrite(pinCC_Right, LOW);
//      digitalWrite(pinCW_Right, HIGH);
//      analogWrite(pinSpeed_Right, awrite); 
//  }
}

//cmdSpeed - [m/s] speed of left wheel
// max cmdSpeed for left wheel ~ 0.52 m/s
void commandLeftWheel(float cmdSpeed){
  cmdSpeed /= MAX_SPEED;
  cmdSpeed *= LIM_CMD_L; 
  if(cmdSpeed > LIM_CMD_L) cmdSpeed = LIM_CMD_L;
  if(cmdSpeed < -LIM_CMD_L) cmdSpeed = -LIM_CMD_L;
  if(fabs(cmdSpeed) < 10){
      digitalWrite(pinCC_Left, LOW);
      digitalWrite(pinCW_Left, LOW);
      analogWrite(pinSpeed_Left, 0);
      return;
  }
//  if(cmdSpeed > prev_left + (LIM_CMD_L /4.0)) cmdSpeed = prev_left + (LIM_CMD_L/4.0);
//  if(cmdSpeed < prev_left - (LIM_CMD_L /4.0)) cmdSpeed = prev_left - (LIM_CMD_L/4.0);
//  prev_left = cmdSpeed;
  if(cmdSpeed > 0){ // move right wheel cw
      digitalWrite(pinCC_Left, HIGH);
      digitalWrite(pinCW_Left, LOW);
      analogWrite(pinSpeed_Left, fabs(cmdSpeed)); 
  }  else{ // move right wheel ccw
      digitalWrite(pinCC_Left, LOW);
      digitalWrite(pinCW_Left, HIGH);
      analogWrite(pinSpeed_Left, fabs(cmdSpeed)); 
  }
  Serial.print("CMDL: "); Serial.print(cmdSpeed);
//  int awrite = (int)(fabs(cmdSpeed - 0.0743)/0.0024);
//  analogWrite(pinSpeed_Left, awrite); 
//
//  if(cmdSpeed <= 0.0743 && cmdSpeed >= -0.0743){
//      digitalWrite(pinCC_Right, LOW);
//      digitalWrite(pinCW_Right, LOW);
//      analogWrite(pinSpeed_Left, 0);
//  }
//  if(cmdSpeed > 0){ // move left wheel ccw
//      digitalWrite(pinCC_Left, HIGH);
//      digitalWrite(pinCW_Left, LOW);
//  }
//  else{ // move left wheel cw
//      digitalWrite(pinCC_Left, LOW);
//      digitalWrite(pinCW_Left, HIGH);
//  }
}

//leftEnc - [m] accumulated distance traveled by left wheel
//rightEnc - [m] accumulated distance traveled by right wheel
void getEncoderDelta(float *leftEnc, float *rightEnc){
  *leftEnc = PI*D_WHEEL*((float)enc_left_pulses)/(64.0*12.0);
  *rightEnc = PI*D_WHEEL*((float)enc_right_pulses)/(64.0*12.0);
  enc_left_pulses = 0;
  enc_right_pulses = 0;
}
