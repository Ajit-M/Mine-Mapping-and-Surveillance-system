/*
   rosserial Ultrasound Example

   This example is for the HC - SR04 Ultrasound rangers.
*/

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>

ros::NodeHandle  nh;

std_msgs::Float64 range_msg;
ros::Publisher pub_range( "ultrasonic", &range_msg);

float getRange(float duration_func) {
  float distance_func;
  distance_func = (duration_func * 0.034) / 2;
  return distance_func;
}



const int trig_pin = 6;
const int echo_pin = 7;
float threshold ;
float duration ;
float distance ;

float count = 0;
float cpr = 14760;
float distance_1;
float wheel_circumference = 21.98;

void pulse()
{
  if (digitalRead(4) == LOW)
    count++;
}


void setup() {
  nh.initNode();
  nh.advertise(pub_range);
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
  pinMode(2, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  //attachInterrupt(0, pulse, LOW);
  attachInterrupt(0, pulse, RISING);
  Serial.begin(57600);

}

void loop() {
  digitalWrite(trig_pin , LOW);
  delayMicroseconds(10);
  digitalWrite(trig_pin , HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin , LOW);
  duration = pulseIn(echo_pin , HIGH);
  distance = getRange(duration);
  // distance = duration*(0.034/2); //in cm
  Serial.print(distance);
  Serial.print(" ");
  range_msg.data = distance;
  pub_range.publish( &range_msg);
  nh.spinOnce();
      distance_1 = (wheel_circumference*count)/cpr;
   // Serial.println(count);
   
    Serial.println(distance_1);
     Serial.println("\n"); 
 

}
