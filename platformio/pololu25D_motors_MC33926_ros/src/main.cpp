
// This file sets the motor to a certain speed, counts encoder ticks, and converts to angular rotation in RPM and Radians/second


const float baud = 57600; //was 9600
// const float baud = 115200;
// const float baud = 500000;

// Arduino Includes
#include <Arduino.h>
// #include <string.h>
// #include <Wire.h>

//Debugging
// #include "avr8-stub.h"

#include "DualMC33926MotorShield.h"

// unsigned char dir1 = 7;
// unsigned char pwm1 = 9;
// unsigned char fb1 = A0;
// unsigned char dir2 = 8;
// unsigned char pwm2 = 10;
// unsigned char fb2 = A1;
// unsigned char nD2 = 4;
// unsigned char nSF = 12;

// DualMC33926MotorShield md(dir1,pwm1,fb1,dir2,pwm2,fb2,nD2,nSF);
DualMC33926MotorShield md;

// ROS includes
#include <ros.h>
#include <motion_control/pid_commands_data.h>


//C++ Function Declarations
int setMotor(int32_t command[2], int pwm_pin, int in1, int in2);
void stopIfFault();

// ROS Function Declarations
void commandCallback(const motion_control::pid_commands_data &msg);

ros::NodeHandle nh;
// std_msgs::Int64 chatter_msg;

// ros::Publisher chatter_pub("chatter", &chatter_msg);
ros::Subscriber<motion_control::pid_commands_data> command_sub("/pid_commands", &commandCallback);

// Buffers to contain the wheel ang velocity commands (after being sent through a PID block) sent from the onboard computer
//if want to modify this in the future must change each of the elements individually (can use a for loop)
volatile int32_t pid_commands[2] = {0, 0};

void setup() {

  //need this for rosserial serial_node.py
  // Note that this uses the Serial stream, so we cannot use that for anything else
  // open communications
  Serial.begin(baud);
  nh.initNode();

  nh.subscribe(command_sub);
  // nh.advertise(chatter_pub);

  md.init();
}

void loop() {

  // signal the motor
  // Front Left Motor
  md.setM1Speed(pid_commands[0]);
  //Back Right Motor
  md.setM2Speed(pid_commands[1]);
  stopIfFault();
  

  nh.spinOnce();
  //ros rate of 200
  //AVOID DELAYS OR MILLIS AT ALL COSTS
  // delay(5);
}


// Subscriber callback for a command message from the onboard computerpwm
void commandCallback(const motion_control::pid_commands_data &msg){
  for (byte i = 0; i < 2; i++){
    pid_commands[i] = msg.data[i];
  }

}

void stopIfFault()
{
  // if (md.getFault())
  // {
  //   Serial.println("fault");
  //   while(1);
  // }
}

//Leftover Code

/*
**************
#ifdef CHATTER
// char debug_msg_char[50];
// sprintf(debug_msg_char, dtostrf(drive_commands[1], 4, 2, "%f\0"));
// str_msg.data = debug_msg_char;
// chatter.publish( &str_msg );
#endif
***************
*/




