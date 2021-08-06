#ifndef WOMBOT_DRIVE_ENCODERS_HH_
#define WOMBOT_DRIVE_ENCODERS_HH_

#include <Arduino.h>

/*
Pololu 25D Motor Specs:
  - Encoder: 48 pulses per revolution
  - Gearbox: 20.4:1 reduction
  - Ticks per pulse: 1 (single edge detection)
  - Total encoder count ratio = 20.4 * 48 * 1 = 979.2 pulses per wheel revolution
Encoder Wiring TODO UPDATE:
  - Orange :  3.3V or 5V
  - Black  :  GND  
  - Blue   :  Encoder Channel A
  - Yellow :  Encoder Channel B
*/

// doesn't have to be PWM, PWM is only for output
// does it need to be analog, though?
#define DRIVE_ENCODER_0A   2   //left motor
#define DRIVE_ENCODER_0B   3
#define DRIVE_ENCODER_1A    18  // right motor
#define DRIVE_ENCODER_1B    19

// Macro defined ISR for single-interrupt encoder counting
#define CREATE_ENCODER_ISR(instance, index)           \
void driveISR##index(){                               \                                                  
  if (digitalRead(DRIVE_ENCODER_##index##B)){     \
   instance.encoder_ticks[index]++;            \
  }else{                                              \
    instance.encoder_ticks[index]--;                  \                                           
  }                                                   \
}                                            

// Conversion factor from encoder ticks to encoder angle (radians)
#define UPDATE_DRIVE_POS(i) pos[i] = (tmp_encoder_ticks[i]/979.2)*TWO_PI

class DriveEncoders{
public:
static const int encoderA[2] = {2, 8};  //0th position is left motor, 1st position is right motor
static const int encoderB[2] = {3, 9};  
bool isr_on;
    void update(){
        // Copy the value of the encoder ticks array (TESTING REQUIRED)
        noInterrupts();
        memcpy((void*) tmp_encoder_ticks, (void*) encoder_ticks, sizeof(encoder_ticks[0])*2);  //was 2
        interrupts();

        // Update the values in the pos array
        UPDATE_DRIVE_POS(0)*(-1); // Motor direction correction
        UPDATE_DRIVE_POS(1);
        isr_on = true;
    }

    void print(){
      char msg[60];
      sprintf(msg, "Drive readings: (%d, %d)", tmp_encoder_ticks[0], tmp_encoder_ticks[1]);
      Serial.println(msg);
    }

    // Record of the number of encoder ticks counted so far
    volatile int encoder_ticks[2] = {0, 0};

    // Since encoder ticks can always be changed by interrupts
    // we create a buffer to copy the values into before doing 
    // any calculations with them
    int tmp_encoder_ticks[2] = {0, 0};

    // Array of wheel rotations in radians
    float pos[2] = {0, 0};
};

#undef UPDATE_DRIVE_POS

#endif
