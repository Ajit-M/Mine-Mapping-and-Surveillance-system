#include <Adafruit_Sensor.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 2  
DHT dht;
ros::NodeHandle  nh;

std_msgs::Float64 humidity;
std_msgs::Float64 temprature;

ros::Publisher pub_temp("temprature" , &temprature);
ros::Publisher pub_hum("humidity" , &humidity);

void setup(){
 nh.initNode(); //initializing the node we have to run this at every node's start
 nh.advertise(pub_temp); //advertising the node to the roscore
 nh.advertise(pub_temp); //advertising the node to the roscore
 Serial.begin(9600);
 Serial.println("Status\tHumidity (%)\tTemperature (C)\t(F)");
 

}

void loop(){

  delay(dht.getMinimumSamplingPeriod());

 float humidity_sensor = dht.getHumidity();
 float temperature_sensor = dht.getTemperature();

 Serial.print(dht.getStatusString());
  Serial.print("\t");
  Serial.print(humidity, 1);
  Serial.print("\t\t");
  Serial.print(temperature, 1);
  Serial.print("\t\t");
  Serial.println(dht.toFahrenheit(temperature), 1);



}
