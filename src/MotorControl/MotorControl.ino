#include <Servo.h>

Servo BLDCMotor;
Servo servoMotor;

char command[5]; 

void setup(){
  Serial.begin(9600);
  servoMotor.attach(3);
  servoMotor.write(90);
  BLDCMotor.attach(5, 1000, 2000);
  BLDCMotor.writeMicroseconds(1500);
  delay(6000);
}

void loop(){
  while(Serial.available() > 0)
  {

    int size = Serial.readBytes(command, 4);
    command[4] = '\0';
    int val = atoi(command); 
    
    Serial.println(val);
    
    if(val <= 1500 && val >= 1420)
    {
      BLDCMotor.writeMicroseconds(val);
    }
    else if(val > 2000 && val <= 2180)
    {
      servoMotor.write(val - 2000);
    }   
    
  }
}
