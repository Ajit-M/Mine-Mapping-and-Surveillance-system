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

const int trig_pin = 3;
const int echo_pin = 2;
float threshold ;
float duration ;
float distance ;


void setup() {
  nh.initNode();
  nh.advertise(pub_range);
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
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
//  Serial.print(distance);
//  Serial.print(" ");
  range_msg.data = distance;
  pub_range.publish( &range_msg);
  nh.spinOnce();
  delay(1000);

}
