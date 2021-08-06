#ifndef WOMBOT_MOTORS_HH_
#define WOMBOT_MOTORS_HH_

#include <Arduino.h>
#include "DC_Motor.hh"
#include <string.h>

// Used for memcpy to set motor speeds to zero in a timeout situation
static const float zeros[2] = {0, 0};

// Pin mappings for the motor drivers TODO UPDATE
// needs to be PWM
// TODO verify in main scrip that first element of pid coming in is for left motor and second one is for right motor
// 1st element of these correspond to left motor, 2nd element corresponds to right motor

DC_Motor dcMotors[2] =  {  DC_Motor(6, 5, 1), DC_Motor(12, 11, 1) };
// DC_Motor motorLeft(6, 5, 1);

class WombotMotors{
public:
//TODO Move this to private but also make getter and setter functions for this so that we can still manipulate it (once its private) in the main.ino
float drive_speed_cap;
float cmd[2] = {0, 0};
//TODO may need to move these to analog pins on Arduino Mega
static const int dir_pins_1st[2] = {6, 12};   //in1, in3 on L298
static const int dir_pins_2nd[2] = {5, 11};   //in2, in4 on L298
static const int pwm_pins[2] = {7, 13};     //enA, enB on L298  TODO these may need to be analog since a PWM signal is being written to them
    
    WombotMotors(){
        drive_speed_cap = 0; // 0.55;
        // Attach the motors to their respective PWM pins, just doing that in drive function instead
    }

    // Set PWM and Direction on one of the drive motors
    void drive(size_t motor_index, float angVel){   //used to of type size_t for motor_index
        //TODO do some logic (if necessary) to convert pid_commands into a PWM signal from 0 to 255
        //may also need to change this to 
        // motorLeft.set_speed(40);
        // motorLeft.forward_with_set_speed();
        // for debug
        if(motor_index == 0) cmd[0] = angVel;
        else cmd[1] = angVel;
        dcMotors[motor_index].set_speed(angVel);
        if (angVel > 0) {
            dcMotors[motor_index].forward_with_set_speed();
            // digitalWrite(dir_pins_1st[motor_index], HIGH);
            // digitalWrite(dir_pins_2nd[motor_index], LOW);                           // drive motor CW
            // analogWrite(pwm_pins[motor_index], angVel);
        }
        else {
            dcMotors[motor_index].reverse_with_set_speed();
            // digitalWrite(dir_pins_1st[motor_index], LOW);
            // digitalWrite(dir_pins_2nd[motor_index], HIGH);                           // drive motor CCW
            // analogWrite(pwm_pins[motor_index], (int)(-1*angVel));  
        }
    }

    // Stop all the motors
    void kill(){
        for (int i = 0; i < 2; i++){
            dcMotors[i].stop_motor();
            // digitalWrite(dir_pins_1st[i], LOW);
            // digitalWrite(dir_pins_2nd[i], LOW);
            // analogWrite(pwm_pins[i], 0.0);      
        }
    }

private:
    
    
};

#endif 

//Leftover Code 
// char debug_msg[80];
// String debug_msg;
// char tmp[40];
// sprintf(tmp, "Command for Motor %d is %d; ", motor_index, angVel);
// debug_msg = debug_msg + String(tmp);