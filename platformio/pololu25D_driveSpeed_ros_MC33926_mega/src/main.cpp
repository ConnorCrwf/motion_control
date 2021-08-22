
// This file sets the motor to a certain speed, counts encoder ticks, and converts to angular rotation in RPM and Radians/second

//make sure control time is 50 and print time is 500 (10% of it)
// #define USBCON
//debug definitions, uncomment out what you don't want
// #define DEBUG_ARDUINO
#define DEBUG_ROS

const float baud = 57600; //was 9600
// const float baud = 115200;
// const float baud = 500000;


// Arduino Includes
#include <Arduino.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro
// #include <string.h>
#include <Wire.h>
//Debugging
// #include "avr8-stub.h"
#include <Encoder.h>
#define encodPinA1 2 // YELLOW
#define encodPinB1 3 // WHITE
#define encodPinA2    18  // right motor
#define encodPinB2    19

#include "DualMC33926MotorShield.h"

unsigned char dir1 = 7;
unsigned char pwm1 = 9;
unsigned char fb1 = A0;
unsigned char dir2 = 8;
unsigned char pwm2 = 10;
unsigned char fb2 = A1;
unsigned char nD2 = 4;
unsigned char nSF = 12;

DualMC33926MotorShield md(dir1,pwm1,fb1,dir2,pwm2,fb2,nD2,nSF);

// ROS includes
#include <../.pio/libdeps/megaatmega2560/Rosserial Arduino Library/src/ros.h>
#include <../.pio/libdeps/megaatmega2560/Rosserial Arduino Library/src/std_msgs/Int64MultiArray.h>
#include <../.pio/libdeps/megaatmega2560/Rosserial Arduino Library/src/std_msgs/Int32MultiArray.h>
#include <../.pio/libdeps/megaatmega2560/Rosserial Arduino Library/src/std_msgs/Float32MultiArray.h>
#include <../.pio/libdeps/megaatmega2560/Rosserial Arduino Library/src/std_msgs/Int64.h>

// TODO Incorporate in debug definitions and logic to only test one or both motors/encoders

//time stuff
int dt_ctrl = 0;
int dt_display = 0;
#define LOOPTIME_DISPLAY 500
unsigned long prevT_display;

unsigned long publish_period = 50;
unsigned long prev_time;

//encoder and velocity stuff
const float RPM2RADPERSEC = 0.104719755f;
float wheelRadius = 3.5;
volatile int64_t countLeft = 0; // tick counter, specify countLeft as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
volatile int64_t countRight = 0;
//declared here since it needs to be used by updatePid
// int angVel_rpm_Left = 0;
// int angVel_rpm_Right = 0;
int dir;

struct encoderInfoStruct {
  int64_t angPos_ticks;
  // int64_t angPos_ticksPrev;
  float angPos_rad;
};


//to have their values set to 0 later in setup
encoderInfoStruct encoderInfoLeft;
encoderInfoStruct encoderInfoRight;

// Control Stuff
//TODO change to use micros and try that out
const int angVel_rpm_target_Left = 220;
const int angVel_rpm_target_Right = 200;
// int error = 0;


int pwr_Left = 0;
int pwr_Right = 0;


// Macros
// Dynamically allocate an array in memory
#define allocateArray(type, len) (type*) malloc(sizeof(type)*len)

//C++ Function Declarations
int setMotor(int32_t command[2], int pwm_pin, int in1, int in2);
encoderInfoStruct processEncoderTicks(encoderInfoStruct encoderInfo, int64_t count_temp);
void stopIfFault();

// ROS Function Declarations
void commandCallback(const std_msgs::Int32MultiArray &msg);

ros::NodeHandle nh;
std_msgs::Int64MultiArray pos_ticks;
std_msgs::Float32MultiArray pos_rad;
std_msgs::Int64 chatter_msg;
// std_msgs::Int32MultiArray command_data_fdbk;

ros::Publisher posTicks_pub("/pos_ticks", &pos_ticks);
ros::Publisher posRad_pub("/pos_rad", &pos_rad);
// ros::Publisher command_pub("/command_data_fdbk", &command_data_fdbk);
// ros::Publisher chatter_pub("chatter", &chatter_msg);

ros::Subscriber<std_msgs::Int32MultiArray> command_sub("/pid_commands", &commandCallback);

// Buffers to contain the wheel ang velocity commands (after being sent through a PID block) sent from the onboard computer
//if want to modify this in the future must change each of the elements individually (can use a for loop)
volatile int32_t pid_commands[4] = {0, 1, 0, 1};
// int32_t pid_commands[4];
// unsigned long update_time; 

Encoder leftEnc(encodPinA1, encodPinB1);
Encoder rightEnc(encodPinA2, encodPinB2);


void setup() {



  //need this for rosserial serial_node.py
 // Note that this uses the Serial stream, so we cannot use that for anything else
  // open communications
  Serial.begin(baud);
  // while (!Serial);
  // Wire.begin();
  


  nh.initNode();
  nh.getHardware()->setBaud(baud); //was 115200
  // nh.getHardware()->setBaud(115200);

  nh.advertise(posTicks_pub);
  nh.advertise(posRad_pub);
  nh.subscribe(command_sub);
  // nh.advertise(chatter_pub);
  // nh.advertise(command_pub);

  // pos_ticks.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  pos_ticks.layout.dim = allocateArray(std_msgs::MultiArrayDimension, 1);
  // pos_ticks.layout.dim.append(std_msgs::MultiArrayDimension());
  pos_ticks.layout.dim[0].label = "pos_ticks";
  pos_ticks.layout.dim[0].size = 2;
  pos_ticks.layout.dim[0].stride = 1*2;
  pos_ticks.data = (long long int *)malloc(sizeof(long long int)*2);
  pos_ticks.layout.dim_length = 0;
  pos_ticks.data_length = 2;

  // pos_rad.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  pos_rad.layout.dim = allocateArray(std_msgs::MultiArrayDimension, 1);
  pos_rad.layout.dim[0].label = "pos_radians";
  pos_rad.layout.dim[0].size = 2;
  pos_rad.layout.dim[0].stride = 1*2;
  pos_rad.data = (float *)malloc(sizeof(float)*2);
  pos_rad.layout.dim_length = 0;
  pos_rad.data_length = 2;

  md.init();

}

void loop() {

  chatter_msg.data = 1;

  unsigned long currentMillis = prev_time;

  // Debug Statements
  // Serial.print("current: ");
  // Serial.println(currentMillis);

  dt_ctrl = currentMillis-prev_time;

  // chatter_msg.data = dt_ctrl;
 
  // if (dt_ctrl >= publish_period) //publish every 50 milliseconds
  // {

  countLeft = leftEnc.read();
  countRight = rightEnc.read();
  

  //make sure these are int64 or else it will default to 16 bit and will only go to -32000 and +32000
  int64_t countLeft_temp = 0; 
  int64_t countRight_temp = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    countLeft_temp = countLeft;
    countRight_temp = countRight;
  }

  
  encoderInfoLeft =  processEncoderTicks(encoderInfoLeft, countLeft_temp);
  encoderInfoRight = processEncoderTicks(encoderInfoRight, countRight_temp);
  

  int64_t angPos_ticks[2] = {encoderInfoLeft.angPos_ticks, encoderInfoRight.angPos_ticks};
  float angPos_rad[2] = {encoderInfoLeft.angPos_rad, encoderInfoRight.angPos_rad};

  

  // signal the motor
  // int32_t left_command[2] = {pid_commands[0], pid_commands[1]};
  // int32_t right_command[2] = {pid_commands[2], pid_commands[3]};
  //Front Left Motor
  md.setM1Speed(pid_commands[0]);
  //Back Right Motor
  md.setM2Speed(pid_commands[2]);
  stopIfFault();
  

  // Publish the data
  // int64_t angPos_ticks[2] = {leftEnc.read(), rightEnc.read()};
  for (int i = 0; i < 2; i++){
    pos_ticks.data[i] = angPos_ticks[i];
    pos_rad.data[i] = angPos_rad[i];
  }
  //first element is front left motor encoder's reading
  //second element is back right motor encoders's reading
  //when motors going forward, left encoders ticks/radians decrease and right encoder ticks/radians increase
  // when motors going forward, radians/second is negative (so CW from motor's persepctive or CCW from third party's perspective)
  posTicks_pub.publish(&pos_ticks);
  posRad_pub.publish(&pos_rad);
  

  // chatter_pub.publish(&chatter_msg);

  prev_time = currentMillis;

  nh.spinOnce();
  //ros rate of 200
  //AVOID DELAYS OR MILLIS AT ALL COSTS
  // delay(5);
}


// Subscriber callback for a command message from the onboard computerpwm
void commandCallback(const std_msgs::Int32MultiArray &msg){
  for (byte i = 0; i < 4; i++){
    pid_commands[i] = msg.data[i];
    // command_data_fdbk.data[i] = msg.data[i];
  }
    // command_pub.publish(&command_data_fdbk);
  #ifdef CHATTER
  // char debug_msg_char[50];
  // sprintf(debug_msg_char, dtostrf(drive_commands[1], 4, 2, "%f\0"));
  // str_msg.data = debug_msg_char;
  // chatter.publish( &str_msg );
  #endif

  // update_time = millis();
}

  //TODO may need this to change this to 12 or 24 if only using the rising edge and not both
encoderInfoStruct processEncoderTicks(encoderInfoStruct encoderInfo, int64_t count_temp)  {  
  //to deal with the fact that the interrupt is only picking up every other pulse instead of every pulse
  //due to issue with using CHANGE in interrupt    
  int correctionFactor = 2; 
  // static long angPos_ticksPrev1 = 0;
  encoderInfo.angPos_ticks = count_temp*correctionFactor;  

  // TODO do all the time stuff in ROS on main computer and not on arduino

  // Conversion factor from encoder ticks to encoder angle (radians)
  // TODO: ask Alex how he got his code to work like this for radian position
  // also ask Alex about encoder Rising or Falling and show him Pololu encoder paragraph on pulses

  //ASK ALEX TODO how do I have countLeft go to zero after reaching 32000 insteaod reversing or is that even what I want?
  //convert raw ticks to rotations of output shaft (utilizes CPR and gear ratio to get PPR. we then divide our pulses by the PRR to get # of revs)
  encoderInfo.angPos_rad = (float)(encoderInfo.angPos_ticks/(48*20.4)); 
  //DEBUG TODO Maybe Change this to use a TWO_PI that i define here
  //restrict between 0 and 2Pi
  encoderInfo.angPos_rad = encoderInfo.angPos_rad - TWO_PI * floor( encoderInfo.angPos_rad / TWO_PI );

  //ASK ALEX: if i need this to handle rollover from 32000 back down

  return encoderInfo;
}

void stopIfFault()
{
  if (md.getFault())
  {
    Serial.println("fault");
    while(1);
  }
}

//Leftover Code

/*
**************


  */




