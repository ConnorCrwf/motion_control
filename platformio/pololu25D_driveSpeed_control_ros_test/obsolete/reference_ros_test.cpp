#include <ros.h>
#include <Arduino.h>
#include <string.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
//TODO where did he include the Atmel stuff for digitalRead/Write Fast
#include <drive_encoders.hh>
#include <motors.hh>


#define MAIN
#ifdef MAIN

/*
- Ensure that GND is common to all components in the project
- All encoder power should be connected to 3.5-5V (drive encoders)
- Pinout diagram reference: Arduino Mega find the link
*/

// Dynamically allocate an array in memory
#define allocateArray(type, len) (type*) malloc(sizeof(type)*len)


// =============================================================
// =========================== DEBUG ===========================
// =============================================================

#define MOTOR_POWER           /* Whether or not to drive the motors */
// #define WOMBOT_DRIVE_DEBUG     /* Print drive data to serial monitor */
// #define WOMBOT_MOTOR_DEBUG 0   /* Print the commands sent to the motors to the serial monitor */
#define CHATTER

#if defined WOMBOT_DRIVE_DEBUG || defined WOMBOT_MOTOR_DEBUG
  #define DEBUG
#endif

void commandCallback(const std_msgs::Float32MultiArray &msg);

// =============================================================
// ===================== Set Up ROS Serial =====================
// =============================================================

// Buffers to contain the wheel ang velocity commands (after being sent through a PID block) sent from the onboard computer
static volatile float drive_commands[2] = {0, 0};

// We can't debug and use rosserial at the same time because we're sharing Serial ports
#ifndef DEBUG

ros::NodeHandle nh;
std_msgs::Float32MultiArray drive_data;
std_msgs::Bool isr_debug;
std_msgs::Float32MultiArray command_data_fdbk;

// Publish encoder data to the computer, Subscribe to PWM motor commands
// TODO figure out how to deal with negative values
ros::Publisher drive_pub("drive_data", &drive_data);
ros::Subscriber<std_msgs::Float32MultiArray> command_sub("/pid_commands", &commandCallback);
// ros::Publisher chatter("chatter", &str_msg);
ros::Publisher command_pub("command_data_fdbk", &command_data_fdbk);
ros::Publisher isr_pub("chatter", &isr_debug);

#else

// Prevent a timeout if we aren't using rosserial
#define update_time millis()

#endif


// =============================================================
// ====================== Set Up Encoders ======================
// =============================================================

// Create class instance to manage encoder readings, each instance keeps track of multiple motors (their encoder data)
static DriveEncoders Drive;

// Procedurally generate interrupt service routines for each drive encoder
static CREATE_ENCODER_ISR(Drive, 0);
static CREATE_ENCODER_ISR(Drive, 1);


// ============================================================
// ================= Set Up Motor Controllers =================
// ============================================================

static WombotMotors Motors;
static bool timeout = false;

// =============================================================
// =========================== SETUP ===========================
// =============================================================

// Used to check for timeout if connection is broken
unsigned long update_time; 

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

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  // If we are in debug mode, start the normal Serial monitor instead of Rosserial
  #ifdef DEBUG

  Serial.begin(57600); //was 115200
  
  #else 

  // ROS Setup ========================================================

  // Note that this uses the Serial stream, so we cannot use that for anything else
  nh.initNode();
  nh.getHardware()->setBaud(57600); //was 115200

  /*
  // Left Motor
  pinMode(Motors.pwm_pins[0], OUTPUT);
  pinMode(Motors.dir_pins_1st[0], OUTPUT);
  pinMode(Motors.dir_pins_2nd[0], OUTPUT);

  // Right Motor
  pinMode(Motors.pwm_pins[1], OUTPUT);
  pinMode(Motors.dir_pins_1st[1], OUTPUT);
  pinMode(Motors.dir_pins_2nd[1], OUTPUT);
  */

  //Left Encoders
  // pinMode(Drive.encoderA[0], INPUT_PULLUP);                  // encoder input A
  // pinMode(Drive.encoderB[0], INPUT_PULLUP);                  // encoder input B

  // //Right Encoders
  // pinMode(Drive.encoderA[1], INPUT_PULLUP);                  // encoder input A
  // pinMode(Drive.encoderB[1], INPUT_PULLUP);                  // encoder input B

  // === DRIVE DATA SETUP === //

  // Create a dim field of size 1
  drive_data.layout.dim = allocateArray(std_msgs::MultiArrayDimension, 1);
  drive_data.layout.dim[0].label  = "drive";    // Dimension label
  drive_data.layout.dim[0].size   = 2;          // Dimension length
  drive_data.layout.dim[0].stride = 2;          // Length of this dimension and all following dimensions

  // We have no use for data_offset, so we set it to zero
  drive_data.layout.data_offset = 0;

  // How many elements we are storing in the array
  drive_data.data_length = 2;

  // Attach the message to our drive encoder class instance
  drive_data.data = Drive.pos;


  isr_debug.data = Drive.isr_on;

  command_data_fdbk.layout.dim = allocateArray(std_msgs::MultiArrayDimension, 1);
  command_data_fdbk.layout.dim[0].label  = "drive_cmd";    // Dimension label
  command_data_fdbk.layout.dim[0].size   = 2;          // Dimension length
  command_data_fdbk.layout.dim[0].stride = 2;          // Length of this dimension and all following dimensions
  command_data_fdbk.layout.data_offset = 0;
  command_data_fdbk.data_length = 2;
  command_data_fdbk.data = Motors.cmd;

  nh.advertise(drive_pub);
  nh.subscribe(command_sub);
  nh.advertise(isr_pub);
  nh.advertise(command_pub);

  // END ROS Setup ----------------------------------------------------
  #endif



  // Encoder Setup ===================================================

  // Drive, this is where the digitalReadFast macro in drive_encoders.hh is used
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENCODER_0A), &driveISR0, RISING);
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENCODER_1A), &driveISR1, RISING);

  // END Encoder Setup ------------------------------------------------


  // Motor Controller Setup ===========================================

  Motors.drive_speed_cap = 0.55;

  // END Motor Controller Setup ---------------------------------------
}

// ==============================================================
// ============================ MAIN ============================
// ==============================================================

void loop() {
  //TODO may want to go back to DigitalRead/Write Fast once I figure out the library it's in
  // Update the encoder readings - This updates the ROS messages as well
  Drive.update();
  #ifdef CHATTER
  command_pub.publish( &command_data_fdbk );
  isr_pub.publish( &isr_debug );
  Drive.isr_on = true; 
  #endif

  #ifdef DEBUG

    #ifdef WOMBOT_DRIVE_DEBUG
    Drive.print();
    #endif
  
  #else

  // Publish the data
  drive_pub.publish(&drive_data);
  nh.spinOnce();

  #endif

  // Check to see that we have not timed out our connection with the host
  timeout = (millis() - update_time > 1000);

  #ifdef MOTOR_POWER

  
  if (!timeout){
    // Control the motors
    for (int i = 0; i < 2; i++){
      Motors.drive(i, drive_commands[i]);
    }
    #ifdef CHATTER
    #endif
    digitalWrite(LED_BUILTIN, LOW);
  }else{
    // Kill motors in timeout event (no new pid_commands received)
    digitalWrite(LED_BUILTIN, HIGH);
    Motors.kill();
  }

  #ifdef WOMBOT_MOTOR_DEBUG
  Serial.print("Command sent to motor "); Serial.print(WOMBOT_MOTOR_DEBUG);
  Serial.print("\n\tDrive: "); Serial.println(drive_commands[WOMBOT_MOTOR_DEBUG]);
  #endif

  #endif

  delay(5); // ~200Hz   was 5
}

#endif // ifdef MAIN

// Leftover Code 1 
// char debug_msg_char[80];
// Motors.debug_msg.toCharArray(debug_msg_char, 80);
// str_msg.data = debug_msg_char;
// Motors.debug_msg = "";
// chatter.publish( &str_msg );

// Leftover Code 2
// String debug_msg;
// char tmp[40];
// sprintf(tmp, "Command for Motor %d is %d; ", i, drive_commands[i]);
// debug_msg = debug_msg + String(tmp);

// Leftover Code 3
// char debug_msg_char[80];
// sprintf(debug_msg_char, "Command for Motor %d is %.2f; ", 0, drive_commands[0]);
// // debug_msg.toCharArray(debug_msg_char, 80);
// str_msg.data = debug_msg_char;
// chatter.publish( &str_msg );

//Leftover Code 4
// char debug_msg_left[50];
// sprintf(debug_msg_left, dtostrf(Motors.vel, 4, 2, "%f\0"));
// str_msg.data = debug_msg_left;
// chatter.publish( &str_msg );