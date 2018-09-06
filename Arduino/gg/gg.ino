#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <TimerOne.h>
#include <r2d2/encoder.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


//defining the Encoder pins
#define LF_enc 30
#define LR_enc 31
#define RF_enc 32
#define RR_enc 34

//defining of the pwm pins
#define LF_pwm_pin 4
#define LR_pwm_pin 5
#define RF_pwm_pin 6
#define RR_pwm_pin 8

//defining the left forward wheel pins
#define LF_DIR 42
#define LF_BRK 43

//defining the left rear wheel pins
#define LR_DIR 44
#define LR_BRK 45

//defining the Right forward wheel pins
#define RF_DIR 46
#define RF_BRK 47


//defining the right rear wheel pins
#define RR_DIR 48
#define RR_BRK 49

#define LOOP_TIME 1000000

MPU6050 mpu;

ros::NodeHandle  nh;

//creating a encoder data message handler
r2d2::encoder encoder_data;
ros::Publisher pub_range( "encoder", &encoder_data);

//geometry_msgs::TransformStamped t;
//tf::TransformBroadcaster broadcaster;

geometry_msgs::Quaternion orientation;
ros::Publisher imu_ori("imu_orientation", &orientation);

geometry_msgs::Vector3 angular_velocity;
ros::Publisher imu_gyr("imu_gyro", &angular_velocity);

geometry_msgs::Vector3 linear_acceleration;
ros::Publisher imu_acc("imu_accl", &linear_acceleration);

//char frameid[] = "/base_link";
//char child[] = "/imu_frame";


float cpr = 14760;
float wheel_circumference = 22;

//variables for the keeping the counts

float count_LF = 0;
float count_LR = 0;
float count_RF = 0;
float count_RR = 0;

//variables for storing distance
float distance_LF;
float distance_LR;
float distance_RF;
float distance_RR;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

//Callback function of the subscriber object
void cmdVelCB( const geometry_msgs::Twist& twist)
{
  //creating a pwm variable so that it can be changed dynamically
  int pwm_LF;
  int pwm_LR;
  int pwm_RF;
  int pwm_RR;

  //creating the variables for making the code easy

  float linear_x = twist.linear.x;
  float angular_z = twist.angular.z;
  //creating the test for checking whether correct data is being sent for debugging


  /*

     If the above test fails , check in the ros_lib for the correct data associations with the
    required variable

    If the above test is correct then comment it out

     Making if else if else conditions for setting up the directions and pwm accordingly
     of the motors

  */
  if (linear_x == 0 && angular_z == 0) {

    //setting up the directions , setting it LOW though the default is LOW just for assurance
    digitalWrite(LF_DIR, LOW);
    digitalWrite(LR_DIR, LOW);
    digitalWrite(RF_DIR, LOW);
    digitalWrite(RR_DIR, LOW);

    //setting up the pwm , setting it all to zero as bot will be stationary in this situation
    pwm_LF = 0;
    pwm_LR = 0;
    pwm_RF = 0;
    pwm_RR = 0;
  }



  else if (angular_z == 0 && linear_x != 0) {

    //Now in this condition there are two cases that the bot will move forward of backward

    //so two conditions

    //It moves in the forward direction , setting up the direction
    if (linear_x < 0) {
      digitalWrite(LF_DIR, LOW);
      digitalWrite(LR_DIR, LOW);
      digitalWrite(RF_DIR, LOW);
      digitalWrite(RR_DIR, LOW);

      // setting up the pwm of the motors
      pwm_LF = 245;
      pwm_LR = 245;
      pwm_RF = 245;
      pwm_RR = 245;

    }
    //It moves in the backward direction
    else if (linear_x > 0) {
      digitalWrite(LF_DIR, HIGH);
      digitalWrite(LR_DIR, HIGH);
      digitalWrite(RF_DIR, HIGH);
      digitalWrite(RR_DIR, HIGH);


      // setting up the pwm of the motors
      pwm_LF = 245;
      pwm_LR = 245;
      pwm_RF = 245;
      pwm_RR = 245;

    }

    //setting default conditions if the tests fails
    else {
      //setting the direction
      digitalWrite(LF_DIR, LOW);
      digitalWrite(LR_DIR, LOW);
      digitalWrite(RF_DIR, LOW);
      digitalWrite(RR_DIR, LOW);

      //setting up the pwm , setting it all to zero as bot will be stationary in this situation
      pwm_LF = 0;
      pwm_LR = 0;
      pwm_RF = 0;
      pwm_RR = 0;

    }
  }



  //For Spot turning

  else if (angular_z != 0 && linear_x == 0) {

    // if the angular z is positive that means it will rotate in left or anti clockwise direction
    if (angular_z > 0) {

      //setting up the directions
      digitalWrite(LF_DIR, LOW);
      digitalWrite(LR_DIR, LOW);
      digitalWrite(RF_DIR, HIGH);
      digitalWrite(RR_DIR, HIGH);

      //setting up the pwm since its spot turning all pwms will be same
      pwm_LF = 245;
      pwm_LR = 245;
      pwm_RF = 245;
      pwm_RR = 245;

    }

    // if the angular z is negative it will rotate in right or clockwise direction
    else if (angular_z < 0) {
      //setting up the directions
      digitalWrite(LF_DIR, HIGH);
      digitalWrite(LR_DIR, HIGH);
      digitalWrite(RF_DIR, LOW);
      digitalWrite(RR_DIR, LOW);

      //setting up the pwm since its spot turning all pwms will be same
      pwm_LF = 245;
      pwm_LR = 245;
      pwm_RF = 245;
      pwm_RR = 245;

    }
    //setting default conditions if the tests fails
    else {
      //setting the direction
      digitalWrite(LF_DIR, LOW);
      digitalWrite(LR_DIR, LOW);
      digitalWrite(RF_DIR, LOW);
      digitalWrite(RR_DIR, LOW);

      //setting up the pwm , setting it all to zero as bot will be stationary in this situation
      pwm_LF = 0;
      pwm_LR = 0;
      pwm_RF = 0;
      pwm_RR = 0;

    }

  }



  /*

    Here the angular part also comes in to play so the pwm has to be set dynamically for the curved turnings

    Here i have divided the part in two phases
    - One without the PID
    - One with PID

    comment out the unrequired one

  */


  //  else if () {
  //
  //
  //  }

  else {

    //if all the above conditions fails this else condition will set it back the default condition

    //setting the direction
    digitalWrite(LF_DIR, LOW);
    digitalWrite(LR_DIR, LOW);
    digitalWrite(RF_DIR, LOW);
    digitalWrite(RR_DIR, LOW);

    //setting up the pwm , setting it all to zero as bot will be stationary in this situation
    pwm_LF = 0;
    pwm_LR = 0;
    pwm_RF = 0;
    pwm_RR = 0;


  }

  analogWrite(LF_pwm_pin, pwm_LF);
  analogWrite(LR_pwm_pin, pwm_LR);
  analogWrite(RF_pwm_pin, pwm_RF);
  analogWrite(RR_pwm_pin, pwm_RR);



}
//creating the subscriber objecct with a callback function
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", cmdVelCB);



void do_count()
{
  if (digitalRead(24) == LOW)
  {
    count_LF++;
  }
  if (digitalRead(25) == LOW)
  {
    count_LR++;
  }

  if (digitalRead(26) == LOW)
  {
    count_RF++;
  }
  if (digitalRead(27) == LOW)
  {
    count_RR++;
  }

}


void timerIsr()
{
  Timer1.detachInterrupt();
  distance_LF =  count_LF;
  distance_LR =  count_LR;
  distance_RF =  count_RF;
  distance_RR = count_RR;

  encoder_data.distance_LF = distance_LF;
  encoder_data.distance_LR = distance_LR;
  encoder_data.distance_RF = distance_RF;
  encoder_data.distance_RR = distance_RR;
  
  delay(100);
  Timer1.attachInterrupt(timerIsr);
}


void setup() {
nh.initNode();
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  nh.initNode();
  //broadcaster.init(nh);
  nh.advertise(imu_ori);
  nh.advertise(imu_gyr);
  nh.advertise(imu_acc);
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  pinMode(LF_pwm_pin, OUTPUT);
  pinMode(LR_pwm_pin, OUTPUT);
  pinMode(RF_pwm_pin, OUTPUT);
  pinMode(RR_pwm_pin, OUTPUT);

  pinMode(LF_DIR, OUTPUT);
  pinMode(LF_BRK, OUTPUT);

  pinMode(LR_DIR, OUTPUT);
  pinMode(LR_BRK, OUTPUT);

  pinMode(RF_DIR, OUTPUT);
  pinMode(RF_BRK, OUTPUT);

  pinMode(RR_DIR, OUTPUT);
  pinMode(RR_BRK, OUTPUT);

  pinMode(2, INPUT_PULLUP);


  Timer1.initialize(LOOP_TIME);

  attachInterrupt(0, do_count, RISING);
  nh.advertise(pub_range);

  Timer1.attachInterrupt(timerIsr);
}


void loop()
{
  nh.spinOnce();

  pub_range.publish( &encoder_data);
 

  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) {}

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
  }
  else if (mpuIntStatus & 0x01)
  {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);

    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    //            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    orientation.x = q.x;
    orientation.y = q.y;
    orientation.z = q.z;
    orientation.w = q.w;
    imu_ori.publish(&orientation);
    nh.spinOnce();
    delay(100);

    angular_velocity.x = ypr[0];
    angular_velocity.y = ypr[1];
    angular_velocity.z = ypr[2];
    imu_gyr.publish(&angular_velocity);
    nh.spinOnce();
    delay(100);

    linear_acceleration.x = aaReal.x * 1 / 16384. * 9.80665;
    linear_acceleration.y = aaReal.y * 1 / 16384. * 9.80665;
    linear_acceleration.z = aaReal.z * 1 / 16384. * 9.80665;
    imu_acc.publish(&linear_acceleration);
    nh.spinOnce();
    delay(100);

    //            t.header.frame_id = frameid;
    //            t.child_frame_id = child;
    //            t.transform.translation.x = 0.5;
    //            t.transform.rotation.x = q.x;
    //            t.transform.rotation.y = q.y;
    //            t.transform.rotation.z = q.z;
    //            t.transform.rotation.w = q.w;
    //            t.header.stamp = nh.now();
    //            broadcaster.sendTransform(t);
  }
}
