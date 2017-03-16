
#define pinEnc_Left_Enc1   2
#define pinEnc_Left_Enc2   4
#define pinEnc_Right_Enc1  3
#define pinEnc_Right_Enc2  5
#define pinON              6
#define pinCW_Left         7    
#define pinCC_Left         8   
#define pinSpeed_Left      9    
#define pinCW_Right       11  
#define pinCC_Right       12
#define pinSpeed_Right    10

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
  
 // while(digitalRead(pinON) == LOW);      // wait for ON switch  digitalWrite(13,LOW);                     // turn LED off
 // delay(1000);
}

//cmdSpeed - [m/s] speed of right wheel
// max cmdSpeed for right wheel ~ 0.52 m/s
void commandRightWheel(float cmdSpeed){
  
  int awrite = (int)((fabs(cmdSpeed) - 0.0629)/0.0023);
  analogWrite(pinSpeed_Right, awrite); 
  
  if(cmdSpeed <= 0.0629 && cmdSpeed >= -0.0629){
      digitalWrite(pinCC_Right, LOW);
      digitalWrite(pinCW_Right, LOW);
  }
  else if(cmdSpeed > 0){ // move right wheel cw
      digitalWrite(pinCC_Right, LOW);
      digitalWrite(pinCW_Right, HIGH);
  }
  else{ // move right wheel ccw
      digitalWrite(pinCC_Right, HIGH);
      digitalWrite(pinCW_Right, LOW);
  }
}

//cmdSpeed - [m/s] speed of left wheel
// max cmdSpeed for left wheel ~ 0.52 m/s
void commandLeftWheel(float cmdSpeed){

  int awrite = (int)(fabs(cmdSpeed - 0.0743)/0.0024);
  analogWrite(pinSpeed_Left, awrite); 

  if(cmdSpeed <= 0.0743 && cmdSpeed >= -0.0743){
      digitalWrite(pinCC_Right, LOW);
      digitalWrite(pinCW_Right, LOW);
  }
  if(cmdSpeed > 0){ // move left wheel ccw
      digitalWrite(pinCC_Left, HIGH);
      digitalWrite(pinCW_Left, LOW);
  }
  else{ // move left wheel cw
      digitalWrite(pinCC_Left, LOW);
      digitalWrite(pinCW_Left, HIGH);
  }
}

//leftEnc - [m] accumulated distance traveled by left wheel
//rightEnc - [m] accumulated distance traveled by right wheel
void getEncoderDelta(float *leftEnc, float *rightEnc){
  *leftEnc = PI*D_WHEEL*((float)enc_left_pulses)/(64.0*12.0);
  *rightEnc = PI*D_WHEEL*((float)enc_right_pulses)/(64.0*12.0);
  enc_left_pulses = 0;
  enc_right_pulses = 0;
}
