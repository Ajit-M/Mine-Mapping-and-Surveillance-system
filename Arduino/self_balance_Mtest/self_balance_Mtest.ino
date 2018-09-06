#include <ros.h>
//Motor control
#include <TimerOne.h>
#include <geometry_msgs/Twist.h>
//IMU
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define LOOP_TIME 200000


#define LM_ENA 2
#define LM_ENB 7
#define RM_ENA 3
#define RM_ENB 8

#define LM_pwm 6
#define RM_pwm 5

#define LM_DIR 11
#define LM_BRK 12

#define RM_DIR 9
#define RM_BRK 10

//IMU
#define SDA 
unsigned int counter_left = 0;
unsigned int counter_right = 0;
float radius = 0.035;
float pi = 3.1415;
float L = 0.1;

ros::NodeHandle nh;

geometry_msgs::Quaternion orientation;
ros::Publisher imu_ori("imu_orientation",&orientation);

geometry_msgs::Vector3 angular_velocity;
ros::Publisher imu_gyr("imu_gyro",&angular_velocity);

geometry_msgs::Vector3 linear_acceleration;
ros::Publisher imu_acc("imu_accl",&linear_acceleration);

geometry_msgs::Twist odom_vel;
ros::Publisher odomVelPub("/odom_vel", &odom_vel);

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector THIS IS WHAT WE NEED

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}


//CALL BACK Function for geometry twists
void cmdVelCB( const geometry_msgs::Twist& twist)
{
  if(twist.angular.z == 0)
  {
        if(twist.linear.x > 0)
        {
          analogWrite(LM_pwm,255);
          analogWrite(RM_pwm,255);
          
          digitalWrite(LM_DIR, LOW); //LOW is forward
          digitalWrite(RM_DIR, LOW);
          
        }
        else if(twist.linear.x < 0)
        {
          analogWrite(LM_pwm,255);
          analogWrite(RM_pwm,255);
          
          digitalWrite(LM_DIR, HIGH);
          digitalWrite(RM_DIR, HIGH);
          
        }
        else
        {
          analogWrite(LM_pwm,0);
          analogWrite(RM_pwm,0);
        }
  }
  else
  {
      if(twist.angular.z > 0)  //left spot turn
      {
        analogWrite(LM_pwm,255);
        analogWrite(RM_pwm,255);
          
        digitalWrite(LM_DIR, HIGH); //left
        digitalWrite(RM_DIR, LOW);
      }
       else if(twist.linear.z < 0)
       {
          analogWrite(LM_pwm,255);
          analogWrite(RM_pwm,255);
          
          digitalWrite(LM_DIR, LOW);
          digitalWrite(RM_DIR, HIGH);
          
        }
        else
        {
          analogWrite(LM_pwm,0);
          analogWrite(RM_pwm,0);
        }
   }
}

ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", cmdVelCB);

//encoder

void docount_left()  
{
  if (digitalRead(LM_ENA) == digitalRead(LM_ENB)) counter_left+=1;
  else counter_left+=2;
} 

void docount_right()
{
  if (digitalRead(RM_ENA) == digitalRead(RM_ENB)) counter_right+=1;
  else counter_right+=2;
} 

void timerIsr()
{
  Timer1.detachInterrupt();
  float left_wheel_vel = float(counter_left)*2*pi*5/8;
  float right_wheel_vel = float(counter_right)*2*pi*5/8;
  odom_vel.linear.x = radius*(left_wheel_vel + right_wheel_vel)/2;
  odom_vel.linear.y = 0;
  odom_vel.linear.z = 0;
  odom_vel.angular.x = 0;
  odom_vel.angular.y = 0;
  odom_vel.angular.z = radius*(left_wheel_vel + right_wheel_vel)/L;
  odomVelPub.publish(&odom_vel);
  counter_right=0;
  counter_left=0;
  Timer1.attachInterrupt(timerIsr);
}

void setup()
{
  pinMode(LM_pwm, OUTPUT);
  pinMode(RM_pwm, OUTPUT);
  pinMode(RM_DIR, OUTPUT);
  pinMode(RM_BRK, OUTPUT);
  pinMode(LM_DIR, OUTPUT);
  pinMode(LM_BRK, OUTPUT);
  
  pinMode(LM_ENA,INPUT_PULLUP);
  pinMode(LM_ENB,INPUT);  
  pinMode(RM_ENA,INPUT_PULLUP);
  pinMode(RM_ENB,INPUT);
  
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();        
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
  
  Timer1.initialize(LOOP_TIME); 

  attachInterrupt(0,docount_left,RISING);
  attachInterrupt(1,docount_right,RISING);
  Timer1.attachInterrupt(timerIsr);

  nh.initNode();
  nh.subscribe(subCmdVel);
  nh.advertise(imu_ori);
  nh.advertise(imu_gyr);
  nh.advertise(imu_acc);
  nh.advertise(odomVelPub);
  
  mpu.initialize();
  devStatus = mpu.dmpInitialize();    
  if (devStatus == 0) {    
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING); // check again 
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
}

void loop() {
  
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
            
            // display quaternion values in easy matrix form: w x y z
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
            
            linear_acceleration.x = aaReal.x * 1/16384. * 9.80665;
            linear_acceleration.y = aaReal.y * 1/16384. * 9.80665;
            linear_acceleration.z = aaReal.z * 1/16384. * 9.80665;
            imu_acc.publish(&linear_acceleration);
            nh.spinOnce();  
            delay(100);         
    }

 // nh.spinOnce();
}


  


