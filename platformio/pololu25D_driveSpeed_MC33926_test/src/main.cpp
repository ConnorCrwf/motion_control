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

void stopIfFault()
{
  if (md.getFault())
  {
    Serial.println("fault");
    while(1);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Dual MC33926 Motor Shield");
  md.init();
}

void loop()
{
  //right side
  md.setM1Speed(90);
  md.setM2Speed(90);
  stopIfFault();
  Serial.print("M1 current: ");
  Serial.println(md.getM1CurrentMilliamps());
  Serial.print("M2 current: ");
  Serial.println(md.getM2CurrentMilliamps());
  delay(2);
  /*
  for (int i = 0; i <= 400; i++)
  {
    md.setM1Speed(i);
    md.setM2Speed(i);
    stopIfFault();
    if (abs(i)%200 == 100)
    {
      Serial.print("M1 current: ");
      Serial.println(md.getM1CurrentMilliamps());
      Serial.print("M2 current: ");
      Serial.println(md.getM2CurrentMilliamps());
    }
    delay(2);
  }
  
  for (int i = 400; i >= -400; i--)
  {
    md.setM1Speed(i);
    md.setM2Speed(i);
    stopIfFault();
    if (abs(i)%200 == 100)
    {
      Serial.print("M1 current: ");
      Serial.println(md.getM1CurrentMilliamps());
      Serial.print("M2 current: ");
      Serial.println(md.getM2CurrentMilliamps());
    }
    delay(2);
  }
  
  for (int i = -400; i <= 0; i++)
  {
    md.setM1Speed(i);
    md.setM2Speed(i);
    stopIfFault();
    if (abs(i)%200 == 100)
    {
      Serial.print("M1 current: ");
      Serial.println(md.getM1CurrentMilliamps());
      Serial.print("M2 current: ");
      Serial.println(md.getM2CurrentMilliamps());
    }
    delay(2);
  }
  */

}