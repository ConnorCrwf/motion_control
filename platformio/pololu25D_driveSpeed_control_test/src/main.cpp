// This file sets the motor to a certain speed, counts encoder ticks, and converts to angular rotation in RPM and Radians/second

#include <Arduino.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <string.h>

#define encodPinA1 2 // YELLOW
#define encodPinB1 3 // WHITE
#define PWM_LEFT 7
#define IN2 5
#define IN1 6
#define encodPinA2    18  // right motor
#define encodPinB2    19
#define PWM_RIGHT 13
#define IN3 12
#define IN4 11

// TODO Incorporate in debug definitions and logic to only test one or both motors/encoders

//time stuff
int prevT_print = 0;
int prevT_ctrl = 0;
int curT = 0;
int LOOPTIME_CTRL = 100;
int LOOPTIME_PRINT = 1000;
int dt = 0;

//encoder and velocity stuff
const float RPM2RADPERSEC = 0.104719755f;
float wheelRadius = 3.5;
volatile int countLeft = 0; // tick counter, specify countLeft as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
volatile int countRight = 0;
//declared here since it needs to be used by updatePid
// int angVel_rpm_Left = 0;
// int angVel_rpm_Right = 0;
int dir;

struct encoderInfoStruct {
  int angVel_rpm;
  long countPrev;
};

//to have their values set to 0 later in setup
encoderInfoStruct encoderInfoLeft;
encoderInfoStruct encoderInfoRight;

// Control Stuff
//TODO change to use micros and try that out
const int angVel_rpm_target_Left = 50;
const int angVel_rpm_target_Right = 100;
// int error = 0;


float pwr_Left = 0;
float pwr_Right = 0;

//Control Stuff v2
//TODO turn these into a struct

struct controllerStateStruct {
  int e;
  //the rest of these should initialize to zero, if they don't do it in setup
  float dedt;
  float eprev;
  float eintegral;
  int u;
};

controllerStateStruct controllerStateLeft;
controllerStateStruct controllerStateRight;

// PID constants
float Kp =  2.5;                                // PID proportional control Gain
float Kd =  0.3;  
float Ki = 0.1;


//Function Declarations
void readEncoderLeft();
void readEncoderRight();
float setMotor(int u_constrained, int pwm, int in1, int in2);
encoderInfoStruct processEncoderTicks(encoderInfoStruct encoderInfo, int count_temp, int dt);
void getControllerData(controllerStateStruct controllerState, int target, float pwr);
int updatePid(int targetValue, int currentValue);
controllerStateStruct updatePid_v2(int targetValue, int currentValue, int dt, controllerStateStruct controllerState);
void processMotorData(String name, int ticks, int angVel_rpm);
int initialCtrlSignal(controllerStateStruct controllerState);


void setup() {
  Serial.begin(9600);

  encoderInfoLeft.angVel_rpm = 0;
  encoderInfoLeft.countPrev = 0;
  encoderInfoRight.angVel_rpm = 0;
  encoderInfoRight.countPrev = 0;

  controllerStateLeft.u = initialCtrlSignal(controllerStateLeft);
  controllerStateRight.u = initialCtrlSignal(controllerStateRight);

  prevT_print = millis();
  prevT_ctrl = millis();

  pinMode(encodPinA1,INPUT);
  pinMode(encodPinB1,INPUT);
  //TODO try initializing them with high like that one dude did in the Pololu forums
  // attachInterrupt(digitalPinToInterrupt(ENCA),readEncoderLeft,CHANGE);
  digitalWrite(encodPinA1, HIGH);                      // turn on pullup resistor
  digitalWrite(encodPinB1, HIGH);
  attachInterrupt(digitalPinToInterrupt(encodPinA1),readEncoderLeft,FALLING);

  pinMode(encodPinA2,INPUT);
  pinMode(encodPinB2,INPUT);
  digitalWrite(encodPinA2, HIGH);                      // turn on pullup resistor
  digitalWrite(encodPinB2, HIGH);
  attachInterrupt(digitalPinToInterrupt(encodPinA2),readEncoderRight,FALLING);
  
  pinMode(PWM_LEFT,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  pinMode(PWM_RIGHT,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
}

void loop() {

  int countLeft_temp = 0; 
  int countRight_temp = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    countLeft_temp = countLeft;
    countRight_temp = countRight;
  }

  curT = millis() ;
  // static int u = 0; 
  dt = curT - prevT_ctrl;
  if (dt >= LOOPTIME_CTRL) {
    //updates angVel_rpm_temp
    encoderInfoLeft =  processEncoderTicks(encoderInfoLeft, countLeft_temp, dt);
    encoderInfoRight = processEncoderTicks(encoderInfoRight, countRight_temp, dt);
    // control signal
    //one form of mapping
    // u = updatePid(angVel_rpm_target_Left, angVel_rpm);
    controllerStateLeft = updatePid_v2(angVel_rpm_target_Left, encoderInfoLeft.angVel_rpm, dt, controllerStateLeft);
    controllerStateRight = updatePid_v2(angVel_rpm_target_Right, encoderInfoRight.angVel_rpm, dt, controllerStateRight);

    //TODO add in condition to check for total elapsed time

    // signal the motor
    pwr_Left = setMotor(controllerStateLeft.u,PWM_LEFT,IN1,IN2);
    // delay(10);
    pwr_Right = setMotor(controllerStateRight.u,PWM_RIGHT,IN4,IN3);

    //reset prev time to know when to go back in loop
    prevT_ctrl = curT;
  }

  //Print out Results
  dt = curT - prevT_print;
  if (dt >= LOOPTIME_PRINT) {
    processMotorData("left", countLeft_temp, encoderInfoLeft.angVel_rpm);
    getControllerData(controllerStateLeft, angVel_rpm_target_Left, pwr_Left);

    processMotorData("right",countRight_temp, encoderInfoRight.angVel_rpm);
    getControllerData(controllerStateRight, angVel_rpm_target_Right, pwr_Right);
    prevT_print = curT;
  }
  
  // delay(5);
}

float setMotor(int u_temp, int pwm, int in1, int in2){
  // motor power
  float pwr = fabs(u_temp);
  //another layer of mapping
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  dir = 1;
  // if(u_temp<0){
  //   dir = -1;
  // }

  analogWrite(pwm,pwr);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
  return pwr;
}

void readEncoderLeft(){
  int b = digitalRead(encodPinB1);
  if(b > 0){
    countLeft++;
  }
  else{
    countLeft--;
  }
}

void readEncoderRight(){
  int b = digitalRead(encodPinB2);
  if(b > 0){
    countRight++;
  }
  else{
    countRight--;
  }
}

  //TODO may need this to change this to 12 or 24 if only using the rising edge and not both
encoderInfoStruct processEncoderTicks(encoderInfoStruct encoderInfo, int count_temp,int dt)  {                                                        // calculate speed, volts and Amps
  // static long countPrev1 = 0;                                                   // last count
  // Convert from ticks to revolutions of output shaft: 48 CPR (pulses) X 75 gear ratio = output shaft revolutions                    
  encoderInfo.angVel_rpm = ((count_temp - encoderInfo.countPrev) * (60 * (1000 / dt))) / (48 * 20.4); 
  //reset count to zero
  // countLeft = 0;
  encoderInfo.countPrev = count_temp;
  return encoderInfo;
}

void processMotorData(String name, int ticks, int angVel_rpm) {
  float angVel_rad = angVel_rpm * RPM2RADPERSEC;
  float linVel = angVel_rad*wheelRadius;
  Serial.print("Motor Side is: ");
  Serial.println(name);
  Serial.print("Encoder Position: ");
  Serial.println(ticks);
  Serial.print("RPM1: ");
  Serial.println(angVel_rpm);
  // Serial.print("Rad/second: ");
  // Serial.println(angVel_rad);
  // Serial.print("Meters/second: ");
  // Serial.println(linVel);
  Serial.print("Direction: ");
  Serial.println(dir);

}

void getControllerData(controllerStateStruct controllerState, int target, float pwr) {
  Serial.print("Error: ");
  Serial.println(controllerState.e);
  Serial.print("Target: ");
  Serial.println(target);
  Serial.print("Control Signal: ");
  Serial.println(controllerState.u);
  // Serial.print("Constrained Control Signal: ");
  // Serial.println(u_constrained);
  Serial.print("PWM: ");
  Serial.println(pwr);
  Serial.println("");
}

// int updatePid(int targetValue, int currentValue)   {             // compute PWM_LEFT value
//   static int last_error = 0;
//   error = targetValue - currentValue;
//   u = (int)((Kp * error) + (Kd * (error - last_error)));
//   last_error = error;
//   if(u < 0) {
//     u = constrain(u, -255, 0);
//   }
//   else {
//     int u = constrain(u, 0, 255);
//   }
//   return u;
// }

controllerStateStruct updatePid_v2(int targetValue, int currentValue, int dt, controllerStateStruct controllerState)   
{ 
  // error
  controllerState.e = targetValue - currentValue;
  // derivative
  controllerState.dedt = (controllerState.e-controllerState.eprev)/(dt);
  // integral
  controllerState.eintegral = controllerState.eintegral + controllerState.e*dt;
  // control signal
  controllerState.u = (int)(Kp*controllerState.e + Kd*controllerState.dedt + Ki*controllerState.eintegral);
  // store previous error
  controllerState.eprev = controllerState.e;
  // if(u < 0) {
  //   u = constrain(u, -255, 0);
  // }
  // else {
  //   int u = constrain(u, 0, 255);
  // }
  return controllerState;
}


//change this to u to a different name
int initialCtrlSignal(controllerStateStruct controllerState)
{
  if (angVel_rpm_target_Left == 30)
  { controllerState.u = 160;
  }

  if (angVel_rpm_target_Left == 45)
  { controllerState.u = 190;
  }

  if (angVel_rpm_target_Left == 60)
  { controllerState.u = 240;
  }

  if (angVel_rpm_target_Left == 75)
  { controllerState.u = 300;
  }

  if (angVel_rpm_target_Left == 90)
  { controllerState.u = 335;
  }

  if (angVel_rpm_target_Left == 105)
  { controllerState.u = 370;
  }

  if (angVel_rpm_target_Left == 120)
  { controllerState.u = 399;
  }
  return controllerState.u;

}


//Leftover Code

// getControllerData1(error, angVel_rpm_target_Left, u);