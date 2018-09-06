#include <LiquidCrystal.h>

int input=12;
int i = 0;
int high_time;
int low_time;
float time_period;
float frequency, rpm, test, r;
float samples[10];

void setup(){
  Serial.begin(9600);
  pinMode(input,INPUT_PULLUP);
}

void loop(){
  high_time=pulseIn(input,HIGH);
  low_time=pulseIn(input,LOW);
   
  time_period=high_time+low_time;
  time_period=time_period/1000;
  if(time_period != 0){
    frequency=1000/time_period;
    r = frequency/15;
    avg(r);
  }
  delay(500);
}

void avg(float r){
  if(i == 10){i =0;}  
  samples[i] = r;
  i++;
  for(int j = 0; j<10; j++){
    test += (samples[j]*0.9);  // interpolation ( -10%)
  }
  rpm = (test/10);
  test =0;
  Serial.println((int)rpm);
}

