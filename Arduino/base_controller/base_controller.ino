/*
   If you are not interested in reading the comments then you are fucked :P
  -Checking for setting up the correct pwm frequncy of the pins
  -Checking the proper polarity of the motors for proper moments of the bot


*/

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <r2d2/encoder.h>


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

//creating the ros node handler
ros::NodeHandle nh;

//creating a encoder data message handler
r2d2::encoder encoder_data;
ros::Publisher pub_range( "encoder", &encoder_data);

//Creating the variables depending on the bots dimensions for encoders
/*
  Wheel dimensions
    Tyre outer diameter: Approx. 120mm/ 4.72inch
    Tyre width: Approx. 70mm/ 2.75inch
    Wheel rim diameter: Approx. 58mm/ 2.28inch
    Hexagonal adapter: Approx. 12mm/0.47inch
    wheel circumference = 37.69908 pi*d (d is outer diameter)

*/
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

//functions that counts the rotations

void do_count_left()
{
  if (digitalRead(24) == LOW)
  {
    count_LF++;
  }
  if (digitalRead(25) == LOW)
  {
    count_LR++;
  }

}


void do_count_right()
{
  if (digitalRead(26) == LOW)
  {
    count_RF++;
  }
  if (digitalRead(27) == LOW)
  {
    count_RR++;
  }
}



void setup() {
  // put your setup code here, to run once:
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
  attachInterrupt(0, do_count_left, RISING);


  pinMode(3, INPUT_PULLUP);
  attachInterrupt(1, do_count_right, RISING);

  nh.initNode();
  nh.subscribe(subCmdVel);
  nh.advertise(pub_range);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  distance_LF =  count_LF;
  distance_LR =  count_LR;
  distance_RF =  count_RF;
  distance_RR = count_RR;
  
  encoder_data.distance_LF = distance_LF;
  encoder_data.distance_LR = distance_LR;
  encoder_data.distance_RF = distance_RF;
  encoder_data.distance_RR = distance_RR;
  pub_range.publish( &encoder_data);
  
  nh.spinOnce();
}
