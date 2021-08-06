// This file sets the motor to a certain speed and counts encoder ticks

#include <Arduino.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM 7
#define IN2 5
#define IN1 6

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long checkT = 0; 

void readEncoder();
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  //TODO try initializing them with high like that one dude did in the Pololu forums
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  // attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,CHANGE);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  
  Serial.println("target pos");
}

void loop() {

  long currT = micros();
  //conver to seconds

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }

  int pwr = 110;

  int dir = 1;

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);

  float deltaCheckT = ((float) (currT - checkT))/( 1.0e6 );
  if(deltaCheckT > 0.500)
    {
      checkT = currT;
      Serial.print("Encoder Position: ");
      Serial.println(pos);
      // Serial.print("deltaCheckT: ");
      // Serial.println(deltaCheckT);
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
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}