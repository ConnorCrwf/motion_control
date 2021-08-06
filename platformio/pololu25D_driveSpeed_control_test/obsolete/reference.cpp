//*************************************************************/
#include <Wire.h>
#include <SPI.h>
#include <DualMC33926MotorShield.h>

#define encodPinA1      2                       // encoder A pin motor 1
#define encodPinB1      6                       // encoder B pin motor 1
#define encodPinA2      3                       // encoder A pin motor 2
#define encodPinB2      5                       // encoder A pin motor 2

//Max K=400
//float Kp =  0.4;                             // PID proportional control Gain
//float Kd =  1.5;                                // PID Derivitave control gain

float Kp =  0.8;                                // PID proportional control Gain
float Kd =  1.5;                                // PID Derivitave control gain

int speed_req = 120;
int e_speed_sum;
float pidTerm = 0;
int error = 0;
int last_error = 0;
int k = 200;
int k_new = 0;
int k1 = k;
int k2 = k;
int t1 = 0;
int t2 = 0;
volatile long count1 = 0;                        // rev counter
volatile long count2 = 0;
int LOOPTIME = 50;
int speed_act1 = 0;                              // speed (actual value)
int speed_act2 = 0;
int current = 0;                                // in mA
int dt = 0;
///About 76 RPM for 1 minute at 300///18
///About 49 ROM for 1 minute at 200///13

///200 45RPM
///175
//125-30RPM
//150-37RPM
DualMC33926MotorShield md;

void stopIfFault()
{
  if (md.getFault())
  {
    Serial.println("fault");
    while (1);
  }
}
void initial_k()
{
  if (speed_req == 30)
  { k1 = 160;
    k2 = 160;
  }

  if (speed_req == 45)
  { k1 = 190;
    k2 = 190;
  }

  if (speed_req == 60)
  { k1 = 240;
    k2 = 240;
  }

  if (speed_req == 75)
  { k1 = 300;
    k2 = 300;
  }

  if (speed_req == 90)
  { k1 = 335;
    k2 = 335;
  }

  if (speed_req == 105)
  { k1 = 370;
    k2 = 370;
  }

  if (speed_req == 120)
  { k1 = 399;
    k2 = 399;
  }

}
void setup () {
  initial_k();
  //Set up Motor Controller
  Serial.begin(115200);
  Serial.println("Dual MC33926 Motor Shield");
  md.init();

  t1 = millis();

  //Set up Encoders
  pinMode(encodPinA1, INPUT);
  pinMode(encodPinB1, INPUT);
  digitalWrite(encodPinA1, HIGH);                      // turn on pullup resistor
  digitalWrite(encodPinB1, HIGH);
  attachInterrupt(0, rencoder1, FALLING);

  pinMode(encodPinA2, INPUT);
  pinMode(encodPinB2, INPUT);
  digitalWrite(encodPinA2, HIGH);                      // turn on pullup resistor
  digitalWrite(encodPinB2, HIGH);
  attachInterrupt(1, rencoder2, FALLING);

}

void loop () {
  //Motor A forward
  k1 = constrain(k1, 0, 399);
  k2 = constrain(k2, 0, 399);
  md.setM1Speed(-k1);

  //Motor B backward
  md.setM2Speed(k2);


  t2 = millis() ;
  if (t2 - t1 >= LOOPTIME) {
    getMotorData1();
    getMotorData2();

    k1 = updatePid(k1, speed_req, abs(speed_act1));
    k2 = updatePid(k2, speed_req, abs(speed_act2));

    t1 = t2;

    // Serial.print("k1: ");
    // Serial.println(k1);
  }

}
void rencoder1()  {                                    // pulse and direction, direct port reading to save cycles
  if (digitalRead(encodPinB1) == HIGH)   count1 ++;             // if(digitalRead(encodPinB1)==HIGH)   count ++;
  else                      count1--;                // if (digitalRead(encodPinB1)==LOW)   count --;
}

void rencoder2()  {                                    // pulse and direction, direct port reading to save cycles
  if (digitalRead(encodPinB2) == HIGH)   count2 ++;            // if(digitalRead(encodPinB1)==HIGH)   count ++;
  else                      count2--;                // if (digitalRead(encodPinB1)==LOW)   count --;
}

void getMotorData1()  {                                                        // calculate speed, volts and Amps
  static long countAnt1 = 0;                                                   // last count
  t2 = millis();
  dt = t2 - t1;
  speed_act1 = ((count1 - countAnt1) * (60 * (1000 / dt))) / (48 * 75) * 4.22222; // 48 pulses X 75 gear ratio = 3600 counts per output shaft rev
  current = md.getM1CurrentMilliamps();                                       // motor current
  Serial.print("RPM1: ");
  Serial.println(speed_act1);
  //   Serial.print("k1 ");
  // Serial.println(k1);
  //
  // Serial.print ("M1: ");
  // Serial.println(current);
  count1 = 0;
}
void getMotorData2()  {                                                        // calculate speed, volts and Amps
  static long countAnt2 = 0;                                                   // last count
  t2 = millis();
  dt = t2 - t1;
  speed_act2 = ((count2 - countAnt2) * (60 * (1000 / dt))) / (48 * 75) * 4.22222; // 48 pulses X 75 gear ratio = 3600 counts per output shaft rev
  current = md.getM2CurrentMilliamps();                                       // motor current
  Serial.print("RPM2: ");
  Serial.println(speed_act2);
  //  Serial.print("k2 ");
  // Serial.println(k2);
  //
  // Serial.print ("M2: ");
  // Serial.println(current);
  count2 = 0;
}

int updatePid(int number_k, int targetValue, int currentValue)   {             // compute PWM value
  float pidTerm = 0;                                                            // PID correction
  int error = 0;
  static int last_error = 0;
  error = abs(targetValue) - abs(currentValue);
  pidTerm = (Kp * error) + (Kd * (error - last_error));
  last_error = error;
  return constrain(number_k + int(pidTerm), 0, 399);
}