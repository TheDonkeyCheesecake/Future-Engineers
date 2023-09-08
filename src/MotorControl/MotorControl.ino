#include <Servo.h>

//declare servo objects for DC motor and servo motor
Servo BLDCMotor;
Servo servoMotor;

//char array for storing signals from raspberry pi 
char command[5]; 

//set up function
void setup(){
  
  Serial.begin(115200); //begin Serial communication at baud rate of 115200
  servoMotor.attach(6); //set servo pin
  servoMotor.write(90); //reset servo angle
  BLDCMotor.attach(5, 1000, 2000); //set DC motor pin
  BLDCMotor.writeMicroseconds(1500); //reset DC motor speed
  pinMode(LED_BUILTIN, OUTPUT); //initialize the built in LED for output
  delay(6000);

}

void loop(){
  //if signal is available
  while(Serial.available() > 0)
  {

    //read signal and convert to 4-digit integer
    int size = Serial.readBytes(command, 4);
    command[4] = '\0';
    int val = atoi(command); 
    
    //if signal is within 1000 and 2000, write the signal to DC motor
    if(val <= 2000 && val >= 1000)
    {
      BLDCMotor.writeMicroseconds(val);
    }
    //if signal is within 2001 and 2180, write the signal to servo motor
    else if(val > 2000 && val <= 2180)
    {
      servoMotor.write(val - 2000);
    //if 9999 is the signal, turn the built-in LED on to show car is in waiting state
    } else if(val == 9999) {
      digitalWrite(LED_BUILTIN, HIGH); 
    }
    
  }
}
