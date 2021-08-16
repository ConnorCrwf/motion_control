
// This file sets the motor to a certain speed, counts encoder ticks, and converts to angular rotation in RPM and Radians/second

// #ifndef HAVE_HWSERIAL1
// #define Serial1 if(false)Serial
// #endif  //HAVE_HWSERIAL1

//make sure control time is 50 and print time is 500 (10% of it)
// #define USBCON
// #define USE_USBCON
// #define ROSSERIAL_ARDUINO_TCP
//debug definitions, uncomment out what you don't want
// #define DEBUG_ARDUINO
#define DEBUG_ROS

// const float baud = 9600;
const float baud = 57600; 
// const float baud = 115200;
// const float baud = 500000;


// Arduino Includes
#include <Arduino.h> // ros.h includes ArduinoHardware.h which includes Arduino.h
// #include <util/atomic.h> // For the ATOMIC_BLOCK macro
// #include <string.h>
// #include <Wire.h>
//Debugging
// #include "avr8-stub.h"
// #include <Encoder.h>
#define encodPinA1 2 // YELLOW
#define encodPinB1 3 // WHITE
#define encodPinA2    18  // right motor
#define encodPinB2    19

// #include "DualMC33926MotorShield.h"

unsigned char dir1 = 7;
unsigned char pwm1 = 9;
unsigned char fb1 = A0;
unsigned char dir2 = 8;
unsigned char pwm2 = 10;
unsigned char fb2 = A1;
unsigned char nD2 = 4;
unsigned char nSF = 12;

// DualMC33926MotorShield md(dir1,pwm1,fb1,dir2,pwm2,fb2,nD2,nSF);

// ROS includes
#include <ros.h>
#include <std_msgs/Int64MultiArray.h>
// #include <std_msgs/Int32MultiArray.h>
// #include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int32.h>

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



//to have their values set to 0 later in setup


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

// void stopIfFault();

// ROS Function Declarations
// void commandCallback(const std_msgs::Int32MultiArray &msg);

// class NewHardware : public ArduinoHardware
// {
//   public:
//   NewHardware():ArduinoHardware(&Serial, 57600){};
// };

// ros::NodeHandle_<NewHardware>  nh;

ros::NodeHandle nh;
// ros::NodeHandle_<ArduinoHardware, 1, 2, 512, 512> nh;
std_msgs::Int64MultiArray pos_ticks;
// std_msgs::Float32MultiArray pos_rad;
std_msgs::Int32 chatter_msg;
// std_msgs::Int32MultiArray command_data_fdbk;

ros::Publisher posTicks_pub("/pos_ticks", &pos_ticks);
// ros::Publisher posRad_pub("/pos_rad", &pos_rad);
// ros::Publisher command_pub("/command_data_fdbk", &command_data_fdbk);
ros::Publisher chatter_pub("chatter", &chatter_msg);

// ros::Subscriber<std_msgs::Int32MultiArray> command_sub("/pid_commands", &commandCallback);

// Buffers to contain the wheel ang velocity commands (after being sent through a PID block) sent from the onboard computer
//if want to modify this in the future must change each of the elements individually (can use a for loop)
// volatile int32_t pid_commands[4] = {0, 1, 0, 1};
// int32_t pid_commands[4];
// unsigned long update_time; 

// Encoder leftEnc(encodPinA1, encodPinB1);
// Encoder rightEnc(encodPinA2, encodPinB2);


void setup() {



  //need this for rosserial serial_node.py
 // Note that this uses the Serial stream, so we cannot use that for anything else
  // open communications
  Serial.begin(baud);
  // while (!Serial);
  // Wire.begin();
  


  nh.initNode();
  // nh.getHardware()->setBaud(baud); //was 115200
  // nh.getHardware()->setBaud(115200);

  nh.advertise(posTicks_pub);
  // nh.advertise(posRad_pub);
  // nh.subscribe(command_sub);
  nh.advertise(chatter_pub);
  // nh.advertise(command_pub);

  pos_ticks.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  pos_ticks.layout.dim = allocateArray(std_msgs::MultiArrayDimension, 1);
  // pos_ticks.layout.dim.append(std_msgs::MultiArrayDimension());
  pos_ticks.layout.dim[0].label = "pos_ticks";
  pos_ticks.layout.dim[0].size = 2;
  pos_ticks.layout.dim[0].stride = 1*2;
  pos_ticks.data = (long long int *)malloc(sizeof(long long int)*2);
  pos_ticks.layout.dim_length = 0;
  pos_ticks.data_length = 2;

  // pos_rad.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  // pos_rad.layout.dim = allocateArray(std_msgs::MultiArrayDimension, 1);
  // pos_rad.layout.dim[0].label = "pos_radians";
  // pos_rad.layout.dim[0].size = 2;
  // pos_rad.layout.dim[0].stride = 1*2;
  // pos_rad.data = (float *)malloc(sizeof(float)*2);
  // pos_rad.layout.dim_length = 0;
  // pos_rad.data_length = 2;



}

void loop() {

  chatter_msg.data = 1;

  

  for (int i = 0; i < 2; i++){
    pos_ticks.data[i] = 2;
  }

  chatter_pub.publish(&chatter_msg);
  posTicks_pub.publish(&pos_ticks);

  nh.spinOnce();
  //ros rate of 200
  //AVOID DELAYS OR MILLIS AT ALL COSTS
  // delay(65);
}


// Subscriber callback for a command message from the onboard computerpwm
// void commandCallback(const std_msgs::Int32MultiArray &msg){
//   for (byte i = 0; i < 4; i++){
//     pid_commands[i] = msg.data[i];
//     // command_data_fdbk.data[i] = msg.data[i];
//   }
//     // command_pub.publish(&command_data_fdbk);
//   #ifdef CHATTER
//   // char debug_msg_char[50];
//   // sprintf(debug_msg_char, dtostrf(drive_commands[1], 4, 2, "%f\0"));
//   // str_msg.data = debug_msg_char;
//   // chatter.publish( &str_msg );
//   #endif

//   // update_time = millis();
// }

  //TODO may need this to change this to 12 or 24 if only using the rising edge and not both


// void stopIfFault()
// {
//   if (md.getFault())
//   {
//     Serial.println("fault");
//     while(1);
//   }
// }

//Leftover Code

/*
**************


  */




