#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <RobojaxBTS7960.h>
#include <Ultrasonic.h>

Servo myservo; //Create front axle steering object
Ultrasonic ultrasonic(2); // Create ultrasonic object and attach it to pin 2

const int MInfra = 13; // Output pin for middle infrared sensor
const int LInfra = 4; // Output pin for left infrared sensor
const int RInfra = 12; // Output pin for right infrared sensor

#define LEFT 130 // Max right steering angle
#define RIGHT 67 // Max left steering angle
#define CENTER 97 // Central steering angle 

#define RPWM 3 // define pin 3 for RPWM pin (output)
#define R_EN 8 // define pin 2 for R_EN pin (input)
#define R_IS 5 // define pin 5 for R_IS pin (output)
#define LPWM 6 // define pin 6 for LPWM pin (output)
#define L_EN 9 // define pin 7 for L_EN pin (input)
#define L_IS 11 // define pin 8 for L_IS pin (output)
#define CW 1 //do not change
#define CCW 0 //do not change
#define debug 0 //change to 0 to hide serial monitor debugging information or set to 1 to view

RobojaxBTS7960 motor(R_EN,RPWM,R_IS, L_EN,LPWM,L_IS,debug); // Create object for controlling DC motor

void setup() {
  myservo.attach(10); // Assign pin 10 to steering axle object
  Turn(CENTER); // Turn steering to Center position by default
  motor.begin(); // Enable motor
  pinMode(LInfra, INPUT); // Set left infrared pin to Input
  pinMode(RInfra, INPUT); // Set right infrared pin to Input
  pinMode(MInfra, INPUT); // Set middle infrared pin to Input
  Serial.begin(115200); // Enable serial monitor
  motor.rotate(10, CCW); // Set fixed speed to 10
}

void loop() {
  int sensorValueM = digitalRead(MInfra);
  int sensorValueL = digitalRead(LInfra);
  int sensorValueR = digitalRead(RInfra); // Read the infrareds outputs
  int distance = ultrasonic.MeasureInCentimeters(); // Read the ultrasonic measurements
  Serial.println("Distance: ");
  Serial.println(distance);
  Serial.println(" cm"); // Code for debugging using serial monitor

  if(sensorValueM == LOW && sensorValueR == LOW && sensorValueL == LOW){
    motor.stop();
    delay(100000); // Code responsible for stopping at the end of the track
  }
  else if (sensorValueL == LOW && sensorValueM == LOW){
    Turn(LEFT);
    // Turn right when only right sensor sees white line
  }
  else if (sensorValueL == LOW){
    Turn(LEFT);
    // Turn left when only left sensor sees white line
  }
  else if (sensorValueR == LOW && sensorValueM == LOW){
    Turn(RIGHT);
    // Turn right when only right sensor sees white line
  }
  else if (sensorValueR == LOW){
    Turn(RIGHT);
    // Turn right when only right sensor sees white line
  }
  
  else if (sensorValueM == LOW){
    Turn(CENTER);
    // Go straight when only middle sensor sees white line
  }
  delay(50);
   if(distance < 65){
    AvoidObstacle();
  }
}

void Turn(int direction){
  myservo.write(direction);
  // Function to steer the front axle
}

 void AvoidObstacle(){
   // Avoidance of an obstacle 
   Turn(LEFT);
   delay(1000);
   Turn(RIGHT+7);
 }
