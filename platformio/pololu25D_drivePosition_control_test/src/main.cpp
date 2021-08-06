// This file sets the motor to a certain speed, counts encoder ticks, and converts to angular rotation in RPM and Radians/second

#include <Arduino.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define encodPinA1 2 // YELLOW
#define encodPinB1 3 // WHITE
#define PWM 7
#define IN2 5
#define IN1 6

//time stuff
int prevT_print = 0;
int prevT_ctrl = 0;
int curT = 0;
int LOOPTIME_CTRL = 50;
int LOOPTIME_PRINT = 2000;
int dt = 0;

//encoder and velocity stuff
const float RPM2RADPERSEC = 0.104719755f;
volatile int count1 = 0; // tick counter, specify count1 as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
int angVel_rpm = 0;    
float angVel_rad = 0.0;                          // speed (actual value)
float linVel = 0.0;
float wheelRadius = 3.5; //TODO correct this

// Control Stuff
//TODO change to use micros and try that out
int e = 0;
float dedt;
float eprev = 0;
float eintegral = 0;
float u;
//TODO change this an angular velocity instead of a position
//will need to do some mapping with pwm mapping and max rpm or max angular velocity
//use drive_encoders_test to do that
int target = 15000;
// PID constants
float kp = 1;
float kd = 0.2;  //was .025
float ki = 0.0; // was 0

//Function Declarations
void readEncoder();
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
void getMotorData1(int count_temp, int dt);
void getControllerData1(int e, int target, float u);


void setup() {
  Serial.begin(9600);

  prevT_print = millis();
  prevT_ctrl = millis();

  pinMode(encodPinA1,INPUT);
  pinMode(encodPinB1,INPUT);
  //TODO try initializing them with high like that one dude did in the Pololu forums
  attachInterrupt(digitalPinToInterrupt(encodPinA1),readEncoder,RISING);
  // attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,CHANGE);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
}

void loop() {

  int count1_temp = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    count1_temp = count1;
  }

  curT = millis() ;

  dt = curT - prevT_ctrl;
  if (dt >= LOOPTIME_CTRL) {
    // error
    e = count1_temp - target;
    // derivative
    dedt = (e-eprev)/(dt);
    // integral
    eintegral = eintegral + e*dt;
    // control signal
    u = kp*e + kd*dedt + ki*eintegral;

    // motor power
    float pwr = fabs(u);
    //simple form of mapping
    if( pwr > 255 ){
      pwr = 255;
    }

    // motor direction
    int dir = 1;
    if(u<0){
      dir = -1;
    }

    // signal the motor
    setMotor(dir,pwr,PWM,IN1,IN2);

    // store previous error
    eprev = e;

    //reset prev time to know when to go back in loop
    prevT_ctrl = curT;
  }

  //Print out Results
  dt = curT - prevT_print;
  if (dt >= LOOPTIME_PRINT) {
    getMotorData1(count1_temp, dt);
    getControllerData1(e, target, u);
    prevT_print = curT;
  }
  
  // delay(5);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
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
}

void readEncoder(){
  int b = digitalRead(encodPinB1);
  if(b > 0){
    count1++;
  }
  else{
    count1--;
  }
}

void getMotorData1(int count_temp,int dt)  {                                                        // calculate speed, volts and Amps
  static long countPrev1 = 0;                                                   // last count
  // Convert from ticks to revolutions of output shaft: 48 CPR (pulses) X 75 gear ratio = output shaft revolutions                    
  angVel_rpm = ((count_temp - countPrev1) * (60 * (1000 / dt))) / (48 * 20.4); 
  angVel_rad = angVel_rpm * RPM2RADPERSEC;
  linVel = angVel_rad*wheelRadius;
  Serial.print("Encoder Position: ");
  Serial.println(count_temp);
  // Serial.print("RPM1: ");
  // Serial.println(angVel_rpm);
  // Serial.print("Rad/second: ");
  // Serial.println(angVel_rad);
  // Serial.print("Meters/second: ");
  // Serial.println(linVel);

  //reset count to zero
  // count1 = 0;
  countPrev1 = count_temp;
}

void getControllerData1(int e, int target, float u) {
  Serial.print("Error: ");
  Serial.println(e);
  Serial.print("Target: ");
  Serial.println(target);
  Serial.print("Control Signal: ");
  Serial.println(u);
  Serial.println("");
}



//LEFTOVER CODE

/*
  float deltaCheckT = ((float) (curT - checkT))/( 1.0e3 );
  // Serial.print("deltaCheckT: ");
  // Serial.println(deltaCheckT);
  if(deltaCheckT > 0.500)
    {
      checkT = curT;
      Serial.print("Encoder Position: ");
      Serial.println(count1_temp);
    }
  */