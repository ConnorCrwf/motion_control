// #define USE_USBCON
// This file sets the motor to a certain speed, counts encoder ticks, and converts to angular rotation in RPM and Radians/second

//make sure control time is 50 and print time is 500 (10% of it)

//debug definitions, uncomment out what you don't want
#define DEBUG_ARDUINO
// #define DEBUG_ROS

// Arduino Includes
#include <Arduino.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <string.h>
#include <Wire.h>
//Debugging
#include "avr8-stub.h"


// ROS includes
#include <../.pio/libdeps/megaatmega2560_ros/rosserial_arduino/ros.h>
#include <../.pio/libdeps/megaatmega2560_ros/rosserial_arduino/std_msgs/Int16MultiArray.h>
#include <../.pio/libdeps/megaatmega2560_ros/rosserial_arduino/std_msgs/Float32MultiArray.h>
#include <../.pio/libdeps/megaatmega2560_ros/rosserial_arduino/std_msgs/Int32.h>
#include <../.pio/libdeps/megaatmega2560_ros/rosserial_arduino/std_msgs/String.h>
#include <../.pio/libdeps/megaatmega2560_ros/rosserial_arduino/std_msgs/Bool.h>
// #include <ros.h>
// #include <std_msgs/Int16MultiArray.h>
// #include <std_msgs/Float32MultiArray.h>
// #include <std_msgs/String.h>
// #include <std_msgs/Bool.h>

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
int dt_ctrl = 0;
int dt_display = 0;
#define LOOPTIME_DISPLAY 500
unsigned long prevT_display;

const int publish_period = 50;
unsigned long prev_time;

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
const int angVel_rpm_target_Left = 220;
const int angVel_rpm_target_Right = 200;
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
float Kp =  1.5;                                // PID proportional control Gain
float Kd =  0.3;  
float Ki = 0.1;

// Macros
// Dynamically allocate an array in memory
#define allocateArray(type, len) (type*) malloc(sizeof(type)*len)

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

// ROS Function Declarations
void commandCallback(const std_msgs::Float32MultiArray &msg);

// Buffers to contain the wheel ang velocity commands (after being sent through a PID block) sent from the onboard computer
static volatile float drive_commands[2] = {0, 0};
unsigned long update_time; 

ros::NodeHandle nh;
std_msgs::Float32MultiArray drive_data;
std_msgs::Int32 drive_data_test;
std_msgs::Bool isr_debug;
std_msgs::Float32MultiArray command_data_fdbk;

ros::Publisher drive_pub("drive_data", &drive_data);
ros::Publisher drive_test_pub("drive_data_test", &drive_data_test);
ros::Publisher command_pub("command_data_fdbk", &command_data_fdbk);
ros::Publisher isr_pub("chatter", &isr_debug);

ros::Subscriber<std_msgs::Float32MultiArray> command_sub("/pid_commands", &commandCallback);


void setup() {



  //need this for rosserial serial_node.py
 // Note that this uses the Serial stream, so we cannot use that for anything else
  // open communications
  const float baud = 57600; //was 9600
  // const float baud = 115200;
  Serial.begin(baud);
  while (!Serial);
  Wire.begin();
  

  #if defined DEBUG_ROS
    nh.initNode();
    // nh.getHardware()->setBaud(57600); //was 115200
    // nh.getHardware()->setBaud(115200);

    // nh.advertise(drive_pub);
    nh.advertise(drive_test_pub);
    // nh.advertise(isr_pub);
    // nh.advertise(command_pub);
    // nh.subscribe(command_sub);
  #endif

  // Create Structure for drive_data message
  // dim field of size 1
  //TODO all the values of 2 below here were 4. make sure that's not an issue.
  drive_data.layout.dim = allocateArray(std_msgs::MultiArrayDimension, 1);
  drive_data.layout.dim[0].label  = "drive";    // Dimension label
  drive_data.layout.dim[0].size   = 2;          // Dimension length
  drive_data.layout.dim[0].stride = 2;          // Length of this dimension and all following dimensions
  // We have no use for data_offset, so we set it to zero
  drive_data.layout.data_offset = 0;
  // How many elements we are storing in the array
  drive_data.data_length = 2;
  //TODO set data for this to something in program down below

  // Create Structure for command_data_fdbk message
  command_data_fdbk.layout.dim = allocateArray(std_msgs::MultiArrayDimension, 1);
  command_data_fdbk.layout.dim[0].label  = "drive_cmd";    // Dimension label
  command_data_fdbk.layout.dim[0].size   = 2;          // Dimension length
  command_data_fdbk.layout.dim[0].stride = 2;          // Length of this dimension and all following dimensions
  command_data_fdbk.layout.data_offset = 0;
  command_data_fdbk.data_length = 2;
  //TODO set data for this to something in program down below

  //TODO see if it does find without these initializers
  encoderInfoLeft.angVel_rpm = 0;
  encoderInfoLeft.countPrev = 0;
  encoderInfoRight.angVel_rpm = 0;
  encoderInfoRight.countPrev = 0;

  // prevT_display = millis();
  // prevT_ctrl = millis();

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

  unsigned long currentMillis = millis();

  // Debug Statements
  // Serial.print("current: ");
  // Serial.println(currentMillis);

  dt_ctrl = (int)(currentMillis-prev_time);
 
  if (dt_ctrl >= publish_period) //publish every 50 milliseconds
  {
  
    prev_time = currentMillis + publish_period;

    int countLeft_temp = 0; 
    int countRight_temp = 0; 
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      countLeft_temp = countLeft;
      countRight_temp = countRight;
    }

    // Serial.print("hey: ");
    encoderInfoLeft =  processEncoderTicks(encoderInfoLeft, countLeft_temp, dt_ctrl);
    encoderInfoRight = processEncoderTicks(encoderInfoRight, countRight_temp, dt_ctrl);

    
    // control signal
    //one form of mapping
    // u = updatePid(angVel_rpm_target_Left, angVel_rpm);
    controllerStateLeft = updatePid_v2(angVel_rpm_target_Left, encoderInfoLeft.angVel_rpm, dt_ctrl, controllerStateLeft);
    controllerStateRight = updatePid_v2(angVel_rpm_target_Right, encoderInfoRight.angVel_rpm, dt_ctrl, controllerStateRight);


    #if defined DEBUG_ARDUINO
    //Print out Results
    // get curT again because you can't rely on the prevoius curT since some time has gone by from the functions above
    // unsigned long curT = millis();
    // unsigned long dt_display = (int)(curT - prevT_display);
    // if (dt_display >= LOOPTIME_DISPLAY) {
      // prevT_display = curT + (unsigned long)LOOPTIME_DISPLAY;
      // processMotorData("left", countLeft_temp, encoderInfoLeft.angVel_rpm);
      // Serial.print("RPM1: ");
      // Serial.println(encoderInfoLeft.angVel_rpm);
      // getControllerData(controllerStateLeft, angVel_rpm_target_Left, pwr_Left);

      // processMotorData("right",countRight_temp, encoderInfoRight.angVel_rpm);
      // getControllerData(controllerStateRight, angVel_rpm_target_Right, pwr_Right);
      
    // }
    #endif
  
    //TODO add in condition to check for total elapsed time

    // signal the motor
    pwr_Left = setMotor(controllerStateLeft.u,PWM_LEFT,IN1,IN2);
    // delay(10);
    pwr_Right = setMotor(controllerStateRight.u,PWM_RIGHT,IN4,IN3);
    command_data_fdbk.data[0] = pwr_Left;
    // command_data_fdbk.data[1] = pwr_Right;

    // Publish the data
    #if defined DEBUG_ROS
      drive_data_test.data = encoderInfoLeft.angVel_rpm;
      drive_test_pub.publish(&drive_data_test);
    #endif


  }

  nh.spinOnce();


  

  // delay(65);
}


// Subscriber callback for a command message from the onboard computer
void commandCallback(const std_msgs::Float32MultiArray &msg){
  for (byte i = 0; i < 2; i++){
    drive_commands[i] = msg.data[i];
    
  }
  #ifdef CHATTER
  // char debug_msg_char[50];
  // sprintf(debug_msg_char, dtostrf(drive_commands[1], 4, 2, "%f\0"));
  // str_msg.data = debug_msg_char;
  // chatter.publish( &str_msg );
  #endif

  update_time = millis();
}

float setMotor(int u_temp, int pwm, int in1, int in2){
  // motor power
  float pwr = fabs(u_temp);
  //another layer of mapping
  if( pwr > 255 ){
    pwr = 255;
  }

  //TODO fix this without it jumping around
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
  // encoderInfo.angVel_rpm = ((count_temp - encoderInfo.countPrev) * (60 * (1000 / dt))) / (12 * 20.4); 
  //reset count to zero
  // countLeft = 0;
  encoderInfo.countPrev = count_temp;
  return encoderInfo;
}

//TODO move angVel_red and linVel to encoderInfo struct
void processMotorData(String name, int ticks, int angVel_rpm) {
  // float angVel_rad = angVel_rpm * RPM2RADPERSEC;
  // float linVel = angVel_rad*wheelRadius;
  // Serial.print("Motor Side is: ");
  // Serial.println(name);
  // Serial.print("Encoder Position: ");
  // Serial.println(ticks);
  Serial.print("RPM1: ");
  Serial.println(angVel_rpm);
  // Serial.print("Rad/second: ");
  // Serial.println(angVel_rad);
  // Serial.print("Meters/second: ");
  // Serial.println(linVel);
  // Serial.print("Direction: ");
  // Serial.println(dir);

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

//TODO Update constraint based on max control signal resulting in max rpm
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
  Serial.print("Current: ");
  Serial.println(currentValue);
  Serial.print("Target: ");
  Serial.println(targetValue);
  Serial.print("Error in: ");
  Serial.println(controllerState.e);

  return controllerState;
}


//Leftover Code

/*

//-------
  // publish at rate given by publish_period
  // if((millis()-timer) >= publish_period) {
    // timer_old = timer;
    // timer=millis();
    
    // if (timer > timer_old) {
      
      // drive_pub.publish(&drive_data);
      // drive_test_pub.publish(&drive_data_test);
      // command_pub.publish( &command_data_fdbk );
    // }
  // }
//-------
  
//------
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
//------

  // curT = millis() ;
  // static int u = 0; 
  // dt_ctrl = curT - prevT_ctrl;
  // if (dt_ctrl >= LOOPTIME_CTRL) {
    //updates angVel_rpm_temp

    #if defined DEBUG_ROS
    
    #endif
    // drive_data.data[0] = (float)(encoderInfoLeft.angVel_rpm);
    // drive_data.data[1] = (float)(encoderInfoRight.angVel_rpm);
    // drive_data_test.data = (int)encoderInfoLeft.angVel_rpm;


    //reset prev time to know when to go back in loop
    // prevT_ctrl = curT;
  // }

  #if defined DEBUG_ARDUINO

  #endif

  

//-----------
int updatePid(int targetValue, int currentValue)   {             // compute PWM_LEFT value
  static int last_error = 0;
  error = targetValue - currentValue;
  u = (int)((Kp * error) + (Kd * (error - last_error)));
  last_error = error;
  if(u < 0) {
    u = constrain(u, -255, 0);
  }
  else {
    int u = constrain(u, 0, 255);
  }
  return u;
}
//-------------

// getControllerData1(error, angVel_rpm_target_Left, u);

*/