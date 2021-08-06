#include <ros.h>
#include <rospy_tutorials/Floats.h>
#define encodPinA1      3      //yellow,  Quadrature encoder A pin
#define encodPinB1      4      //white,  Quadrature encoder B pin
#define enA             9      //orange                 // PWM output to motor driver module
#define in1             8     //blue
#define in2             7     //yellow

ros::NodeHandle  nh;

double pos = 0, vel= 0, output = 0, temp=0;
unsigned long lastTime,now,lasttimepub;
volatile long encoderPos = 0,last_pos=0;
rospy_tutorials::Floats joint_state;

void set_angle_cb( const rospy_tutorials::Floats& cmd_msg){
  output= cmd_msg.data[0]; 
}


ros::Subscriber<rospy_tutorials::Floats> sub("/joints_to_aurdino", set_angle_cb);
ros::Publisher pub("/joint_states_from_arduino", &joint_state);

void setup(){
//  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise 

  pinMode(encodPinA1, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);                  // quadrature encoder input B
  attachInterrupt(digitalPinToInterrupt(encodPinA1), updateEncoder, RISING);              // update encoder position
   
}


void loop(){
  pos = (encoderPos*360)/2200 ;
  now = millis();
  int timeChange = (now - lastTime);
  if(timeChange>=500 )
  {
      temp = (360.0*1000*(encoderPos-last_pos)) /(2200.0*(now - lastTime));
      if ((encoderPos < -2 || encoderPos > 2) && temp >= -60 && temp <=60 ) // to gaurd encoderPos at boundary i.e., after max limit it will rest. Then lastPos will be greater than encoderpos
          vel =temp;
      lastTime=now;
      last_pos=encoderPos;
  }

  pwmOut(output);
  
  if ((now - lasttimepub)> 100)
  {
    joint_state.data_length=2;
    joint_state.data[0]=pos;
    joint_state.data[1]=vel;
    pub.publish(&joint_state);
    lasttimepub=now;
  }

  nh.spinOnce();

}

//analog write only works for positive
void pwmOut(float out) {                                
  if (out > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);                           // drive motor CW
    analogWrite(enA, out);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);   
    analogWrite(enA, out);                        // drive motor CCW
  }
}

void updateEncoder()  {                                     // pulse and direction, direct port reading to save cycles

//  Serial.println(encoderPos);
  if (encoderPos > 2250 || encoderPos < -2250)
    encoderPos=0; 
  if(digitalRead(encodPinB1)==HIGH)  encoderPos++;             // if(digitalRead(encodPinB1)==HIGH)   count ++;
  else                      encoderPos--;             // if(digitalRead(encodPinB1)==LOW)   count --;
}
