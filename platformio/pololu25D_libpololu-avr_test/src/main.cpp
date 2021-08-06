#include <Arduino.h>
#include <PololuWheelEncoders.hh>

#define IO_D2				2
#define IO_D3				3
#define IO_C2				18
#define IO_C3				19

#define PWM 7
#define IN2 5
#define IN1 6

static PololuWheelEncoders encoders;

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);

void setup()
{
  // Initialize the encoders and specify the four input pins.
  // encoders.init(PD3, PD2, PE4, PE5);
  encoders.init(IO_D2, IO_D3, IO_C2, IO_C3);
 
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  Serial.begin(9600);
}
 
void loop()
{
  
  int dir = 1;
  int pwr = 101;

  setMotor(dir,pwr,PWM,IN1,IN2);
 
  while(1)
  {
    // Read the counts for motor 1
    Serial.print("Left Encoder counts: ");
    Serial.println(encoders.getCountsM1());
 
    // Read the counts for motor 2
    Serial.print("Right Encoder counts: ");
    Serial.println(encoders.getCountsM2());
 
    // Print encoder errors, if there are any.
    // delay(50);
  }
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