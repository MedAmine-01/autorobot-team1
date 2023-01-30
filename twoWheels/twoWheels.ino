//defining pins
#define enc_left_A 1
#define enc_left_B 2
#define enc_right_A 3
#define enc_right_B 4
#define in_left1 5 
#define in_left2 6
#define in_right1 9
#define in_right2 10
//PID coefficient 
#define kp 0.111
#define kd 0
#define ki 0.0005

//counters for both encoders
long int counterA=0;
long int counterB=0;

//making globale variables for pid , so we can just use it in the function without passing it by address ;
double errSum ;
double lastError; 


void setup() { 
//all pimode configurations    
  pinMode (enc_left_A,INPUT_PULLUP);
  pinMode (enc_left_B,INPUT_PULLUP);
  pinMode (enc_right_A,INPUT_PULLUP);
  pinMode (enc_right_B,INPUT_PULLUP);
  pinMode(in_left1,OUTPUT);  
  pinMode(in_left2,OUTPUT);
  pinMode(in_right1,OUTPUT);
  pinMode(in_right2,OUTPUT);
//Attaching Interrupt to both encoders ;
  attachInterrupt(digitalPinToInterrupt(enc_left_A),ReadEncoder_ISR_left,CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_left_B),ReadEncoderB_left,CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_right_A),ReadEncoder_ISR_right,CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_right_B),ReadEncoderB_right,CHANGE);
  Serial.begin (9600);
}

 
void loop() { 
  
}
  
void ReadEncoder_ISR_left(){
  int a= digitalRead(enc_left_A);
  int b= digitalRead(enc_left_B);
  if(a^b){
    counterA ++;
  }
  else{
    counterA--;
  }
}
void ReadEncoderB_left(){
  int a= digitalRead(enc_left_A);
  int b= digitalRead(enc_left_B);
  if(!(a^b)){
    counterA ++;
  }
  else{
    counterB--;
  }
}

void ReadEncoder_ISR_right(){
  int a= digitalRead(enc_right_A);
  int b= digitalRead(enc_right_B);
  if(a^b){
    counterB ++;
  }
  else{
    counterB--;
  }
}
void ReadEncoderB_right(){
  int a= digitalRead(enc_right_A);
  int b= digitalRead(enc_right_B);
  if(!(a^b)){
    counterB ++;
  }
  else{
    counterB--;
  }
}



void goForward(int goal){
  //Setting motor power 
  int power_left =pidCount(goal,0);
  int power_right =power_left;
  //Used to adjust motors' speed 
  unsigned diff_l ;
  unsigned diff_r ; 
  counterA = 0 ;
  counterB = 0 ;
  //prev counter 
  unsigned long prevCounterA = 0 ; 
  unsigned long prevCounterB = 0 ;
  unsigned long  now =0,lastTime=0 ;
  errSum=0 ;
  lastError=0;   
  while((counterA<goal)&&(counterB<goal)){
    drive(power_left,power_right) ;
    diff_l = counterA - prevCounterA ;
    diff_r = counterB - prevCounterB ;

    prevCounterA = counterA ;
    prevCounterB = counterB ;
    power_left = pidCount(goal,counterA) ;
    power_right = pidCount(goal,counterB) ;
    if(diff_l>diff_r){
      power_left -= power_left*0.05;
      power_right += power_right*0.05 ;    
    }
    else if(diff_l<diff_r){
      power_left += power_left*0.05;
      power_right -= power_right*0.05 ;        
    }
    now = millis();
    delay(20-now+lastTime) ;
    lastTime = now ;    
  }
}

  

  
  double pidCount(int goal,int currentCounter){
    double timeChange = 20
    double error = goal - counter ;
    errSum += error*timeChange ;
    errSum = constrain(errSum,-280.5,280.5) ;
    double dErr = (error -lastError)/timeChange ;
    double output = (kp*error + ki*errSum + kd*dErr) ; 
    output = constrain(output,-255,255);
    lastError = error ;
    return output ;        
}

void drive(int power_a, int power_b) {

  // Constrain power to between -255 and 255
  power_a = constrain(power_a, -255, 255);
  power_b = constrain(power_b, -255, 255);

  // Left motor direction
  if ( power_a < 0 ) {
    analogWrite(in_left2, 0);
    analogWrite(in_left1, power_a);
  } else {
    analogWrite(in_left2, power_a);
    analogWrite(in_left1, 0);
  }

  // Right motor direction
  if ( power_b < 0 ) {
    analogWrite(in_right1, 0);
    analogWrite(in_right2, power_b);
  } else {
    analogWrite(in_right1, power_b);
    analogWrite(in_right2, 0);
  }

}
