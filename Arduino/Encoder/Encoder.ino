
float count = 0;
float cpr = 14760;
float distance;
float wheel_circumference = 21.98;

void pulse()
{
    if(digitalRead(4) == LOW)
      count++;
}

void setup()
{
    Serial.begin(9600);
    pinMode(2, INPUT_PULLUP); 
    pinMode(4, INPUT_PULLUP); 
    //attachInterrupt(0, pulse, LOW); 
    attachInterrupt(0, pulse, RISING); 
}

void loop()
{
    distance = (wheel_circumference*count)/cpr;
   // Serial.println(count);
    Serial.println("\n");
    Serial.println(distance);
    delay(2000);
}   
