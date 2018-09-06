#include <ros.h>
#include <ros/time.h>
#include <practise/custom.h>

// Motor A
int pwm_1 = 5;
int dir_1 = 4;
int brk_1 = 3;

// Motor B
int pwm_2 = 6;
int dir_2 = 7;
int brk_2 = 8;


void setup() {
  // put your setup code here, to run once:
  pinMode(pwm_1 , OUTPUT);
  pinMode(dir_1 , OUTPUT);
  pinMode(brk_1 , OUTPUT);
  pinMode(pwm_2 , OUTPUT);
  pinMode(dir_2 , OUTPUT);
  pinMode(brk_2 , OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(pwm_1 , HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(dir_1 , HIGH);
  delay(2000);
  
   digitalWrite(brk_1 , HIGH);

}
