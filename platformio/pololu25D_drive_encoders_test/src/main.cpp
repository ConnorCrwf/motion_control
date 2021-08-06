// This file sets the motor to a certain speed, counts encoder ticks, and converts to angular rotation in RPM and Radians/second

#include <Arduino.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define encodPinA1 2 // YELLOW
#define encodPinB1 3 // WHITE
#define PWM 7
#define IN2 5
#define IN1 6

const float RPM2RADPERSEC = 0.104719755f;


long checkT = 0; 
int t1 = 0;
int t2 = 0;
int LOOPTIME = 500;

volatile int count1 = 0; // tick counter, specify count1 as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
int angVel_rpm = 0;    
float angVel_rad = 0.0;                          // speed (actual value)
float linVel = 0.0;
float wheelRadius = 3.5; //TODO correct this

int dt = 0;



void readEncoder();
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
void getMotorData1(int count_temp);



void setup() {
  Serial.begin(9600);

  t1 = millis();

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

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int count1_temp = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    count1_temp = count1;
  }

  t2 = millis() ;
  if (t2 - t1 >= LOOPTIME) {
    getMotorData1(count1_temp);

    t1 = t2;
  }

  int pwr = 110;

  int dir = 1;

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);

  /*
  float deltaCheckT = ((float) (t2 - checkT))/( 1.0e3 );
  // Serial.print("deltaCheckT: ");
  // Serial.println(deltaCheckT);
  if(deltaCheckT > 0.500)
    {
      checkT = t2;
      Serial.print("Encoder Position: ");
      Serial.println(count1_temp);
    }
  */


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

void getMotorData1(int count_temp)  {                                                        // calculate speed, volts and Amps
  static long countPrev1 = 0;                                                   // last count
  t2 = millis();
  dt = t2 - t1;

  // Convert from ticks to revolutions of output shaft: 48 CPR (pulses) X 75 gear ratio = output shaft revolutions                    
  angVel_rpm = ((count_temp - countPrev1) * (60 * (1000 / dt))) / (48 * 20.4); 
  angVel_rad = angVel_rpm * RPM2RADPERSEC;
  linVel = angVel_rad*wheelRadius;
  Serial.print("Encoder Position: ");
  Serial.println(count_temp);
  Serial.print("RPM1: ");
  Serial.println(angVel_rpm);
  // Serial.print("Rad/second: ");
  // Serial.println(angVel_rad);
  // Serial.print("Meters/second: ");
  // Serial.println(linVel);

  //reset count to zero
  // count1 = 0;
  countPrev1 = count_temp;
}