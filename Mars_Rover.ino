/*
 *   Libraries:
 *   ServoEasing: https://github.com/ArminJo/ServoEasing
 *   IBusBM: https://github.com/bmellink/IBusBM
 */

#include <Servo.h>
#include <ServoEasing.h>
#include <IBusBM.h>

//IN2 - Give signal high for backward rotation of wheel
//IN1 - Give signal high for forward  rotation of wheel
//Front left
#define motorW1_IN1 6
#define motorW1_IN2 7

//Middle left
#define motorW2_IN1 4
#define motorW2_IN2 5

//Back left
#define motorW3_IN1 2
#define motorW3_IN2 3

//Front right
#define motorW4_IN1 10
#define motorW4_IN2 13

//Middle right
#define motorW5_IN1 8
#define motorW5_IN2 9

//Back right
#define motorW6_IN1 11
#define motorW6_IN2 12

ServoEasing servoW1;  //Servo connected to pin number 22
ServoEasing servoW3;  //Servo connected to pin number 23
ServoEasing servoW4;  //Servo connected to pin number 24
ServoEasing servoW6;  //Servo connected to pin number 25

IBusBM IBus;
IBusBM IBusSensor;

int angle = 0;   // servo position in degrees
int ch0, ch2, ch6 = 0;
int servo1Angle = 90;
int servo3Angle = 90;
int servo4Angle = 90;
int servo6Angle = 90;
int s = 0; // rover speed
int r = 0; // turning radius
int m1, m2, m3, m4, m5, m6;

//speed1 = Speed of the three outer wheels while rotation. This will have the highest value among speed1, speed2, speed3.
//speed2 = Speed of the inner front and back wheels.
//speed3 = Speed of the inner middle wheel. This will have the lowest value among speed1, speed2, speed3.
float speed1, speed2, speed3 = 0;

//Depending on the extent to which the user bends the joystick, the analog write(0-255) correspondingly will be calculated and stored in these variables for the three speends declared in the previous line(speed1, speed2 and speed3). First used in "calculateMotorSpeed()" 
float speed1PWM, speed2PWM, speed3PWM = 0;

//Angle of rotation of the two front and back wheels
float thetaInnerFront, thetaInnerBack, thetaOuterFront, thetaOuterBack = 0;

float d1 = 271; // Distance in mm
float d2 = 278;
float d3 = 301;
float d4 = 304;


void setup() {

  /*
     Use this if you need to change the frequency of the PWM signals
    TCCR4B = TCCR4B & B11111000 | B00000101;     // D6,D7,D8 PWM frequency of 30.64 Hz
    TCCR2B = TCCR2B & B11111000 | B00000111;   // D9, D10 PWM frequency of 30.64 Hz
    TCCR1B = TCCR1B & B11111000 | B00000101;   // D11, D12  PWM frequency of 30.64 Hz
    TCCR5B = TCCR5B & B11111000 | B00000101; // D4, D13 PWM frequency of 30.64 Hz
    TCCR3B = TCCR3B & B11111000 | B00000101;    // D2, D3, D5 PWM frequency of 30.64 Hz
  */
  
  Serial.begin(115200);
  IBus.begin(Serial1, IBUSBM_NOTIMER); // Servo iBUS
  IBusSensor.begin(Serial2, IBUSBM_NOTIMER); // Sensor iBUS
  IBusSensor.addSensor(IBUSS_INTV); // add voltage sensor

  servoW1.attach(22);
  servoW3.attach(23);
  servoW4.attach(24);
  servoW6.attach(25);

  servoW1.write(90);
  servoW3.write(90);
  servoW4.write(90);
  servoW6.write(90);

  servoW1.setSpeed(550);
  servoW3.setSpeed(550);
  servoW4.setSpeed(550);
  servoW6.setSpeed(550);

  // Left Front
  digitalWrite(motorW1_IN1, LOW);
  digitalWrite(motorW1_IN2, LOW); // Forward

  // Left Middle
  digitalWrite(motorW2_IN1, LOW);
  digitalWrite(motorW2_IN2, LOW);
  
  //Left Back
  digitalWrite(motorW3_IN1, LOW);
  digitalWrite(motorW3_IN2, LOW);
  
  //Right Front
  digitalWrite(motorW4_IN1, LOW);
  digitalWrite(motorW4_IN2, LOW);
  
  //Right Middle
  digitalWrite(motorW5_IN1, LOW);
  digitalWrite(motorW5_IN2, LOW);
  
  //Right Back
  digitalWrite(motorW6_IN1, LOW);
  digitalWrite(motorW6_IN2, LOW);


}

void loop() {

  IBus.loop();
  ch0 = IBus.readChannel(0);
  ch2 = IBus.readChannel(2);
  ch6 = IBus.readChannel(6);

  // Steering right
  if (ch0 > 1515) {
    r = map(ch0, 1515, 2000, 1400, 600); // Turning radius from 1400mm to 600mm
  }
  // Steering left
  else if (ch0 < 1485) {
    r = map(ch0, 1485, 1000, 1400, 600); // Turning radius from 600mm to 1400mm
  }

  // Rover speed in % from 0 to 100
  s = map(ch2, 1000, 2000, 0, 100); 

  calculateMotorsSpeed(); //Function to calculate the values of speed1PWM, speed2PWM and speed3PWM.
  calculateServoAngle();  //Function to calculate the angle by which the servos will rotate. Calculates the value of thetaInnerFront, thetaInnerBack, thetaOuterFront and thetaOuterBack.

  // To steer the rover to the right.
  if (ch0 > 1515) {
    
    // Outer wheels
    servoW1.startEaseTo(97 + thetaInnerFront);  // Front wheel steer right
    servoW3.startEaseTo(97 - thetaInnerBack);   // Back wheel steer left for overall steering to the right of the rover
    
    // Inner wheels
    servoW4.startEaseTo(94 + thetaOuterFront);
    servoW6.startEaseTo(96 - thetaOuterBack);

    //To move forward and right.
    if (ch6 < 1500) {

      //Left Front
      analogWrite(motorW1_IN1, speed1PWM);   // Outer wheels running at speed1 - max speed.
      digitalWrite(motorW1_IN2, LOW);

      //Left Middle
      analogWrite(motorW2_IN1, speed1PWM);
      digitalWrite(motorW2_IN2, LOW);

      //Left Back
      analogWrite(motorW3_IN1, speed1PWM);
      digitalWrite(motorW3_IN2, LOW);

      // Right side motors move in opposite direction so that collectively both the sides rotate in such a manner that the rover moves in the same direction.

      //Right Front
      digitalWrite(motorW4_IN1, LOW);
      analogWrite(motorW4_IN2, speed2PWM); // Inner front wheel running at speed2 - lower speed

      //Right Middle
      digitalWrite(motorW5_IN1, LOW);
      analogWrite(motorW5_IN2, speed3PWM); // Inner middle wheel running at speed3 - lowest speed

      //Right Back
      digitalWrite(motorW6_IN1, LOW);
      analogWrite(motorW6_IN2, speed2PWM); // Inner back wheel running at speed2 - lower speed
    }

    //To move backward and right.
    else if (ch6 > 1500) {
      
      //Left Front
      digitalWrite(motorW1_IN1, LOW);   // Outer wheels running at speed1 - max speed
      analogWrite(motorW1_IN2, speed1PWM);

      //Left Middle
      digitalWrite(motorW2_IN1, LOW);
      analogWrite(motorW2_IN2, speed1PWM);

      //Left Back
      digitalWrite(motorW3_IN1, LOW);
      analogWrite(motorW3_IN2, speed1PWM);

      // Right side motors move in opposite direction so that collectively both the sides rotate in such a manner that the rover moves in the same direction.

      //Right Front
      analogWrite(motorW4_IN1, speed2PWM);
      digitalWrite(motorW4_IN2, LOW); // Inner front wheel running at speed2 - lower speed

      //Right Middle
      analogWrite(motorW5_IN1, speed3PWM);
      digitalWrite(motorW5_IN2, LOW); // Inner middle wheel running at speed3 - lowest speed

      //Right Back
      analogWrite(motorW6_IN1, speed2PWM);
      digitalWrite(motorW6_IN2, LOW); // Inner back wheel running at speed2 - lower speed
    }
  }

  // Steer left
  else if (ch0 < 1485) {
    servoW1.startEaseTo(97 - thetaOuterFront);
    servoW3.startEaseTo(97 + thetaOuterBack);
    servoW4.startEaseTo(94 - thetaInnerFront);
    servoW6.startEaseTo(96 + thetaInnerBack);

     // To move forward and left
    if (ch6 < 1500) {

      //Left Front
      analogWrite(motorW1_IN1, speed2PWM);    // speed2PWM value since now the left side wheels are the inner wheels so they will have lower speed as compared to the outer right wheels.
      digitalWrite(motorW1_IN2, LOW); 
      
      //Left Middle
      analogWrite(motorW2_IN1, speed3PWM);    // speed3PWM value since this is the inner middle wheel and it will have the lowest speed.
      digitalWrite(motorW2_IN2, LOW);

      //Left Back
      analogWrite(motorW3_IN1, speed2PWM);
      digitalWrite(motorW3_IN2, LOW);

      // Right side motors move in opposite direction

      //Right Front      
      digitalWrite(motorW4_IN1, LOW);
      analogWrite(motorW4_IN2, speed1PWM);    //Since all the right side wheels are outer wheels while steering left, all of them will have the same and maximum speed, speed1PWM

      //Right Middle
      digitalWrite(motorW5_IN1, LOW);
      analogWrite(motorW5_IN2, speed1PWM);

      //Right Back
      digitalWrite(motorW6_IN1, LOW);
      analogWrite(motorW6_IN2, speed1PWM);
    }

     // Move backward and left
    else if (ch6 > 1500) {

      //Left Front
      digitalWrite(motorW1_IN1, LOW);
      analogWrite(motorW1_IN2, speed2PWM);

      //Left Middle
      digitalWrite(motorW2_IN1, LOW);
      analogWrite(motorW2_IN2, speed3PWM);

      //Left Back
      digitalWrite(motorW3_IN1, LOW);
      analogWrite(motorW3_IN2, speed2PWM);

      // Right side motors move in opposite direction

      //Right Front
      analogWrite(motorW4_IN1, speed1PWM);
      digitalWrite(motorW4_IN2, LOW);

      //Right Middle
      analogWrite(motorW5_IN1, speed1PWM);
      digitalWrite(motorW5_IN2, LOW);

      //Right Back
      analogWrite(motorW6_IN1, speed1PWM);
      digitalWrite(motorW6_IN2, LOW);
    }
  }

  //No steering, just move straight. There will be no concept of "inner wheels" or "outer wheels" here so all motors will rotate at the same speed.
  else {
    servoW1.startEaseTo(97);
    servoW3.startEaseTo(97);
    servoW4.startEaseTo(94);
    servoW6.startEaseTo(96);

    if (ch6 < 1500) {
      //Left Front
      analogWrite(motorW1_IN1, speed1PWM);
      digitalWrite(motorW1_IN2, LOW);

      //Left Middle
      analogWrite(motorW2_IN1, speed1PWM);
      digitalWrite(motorW2_IN2, LOW);

      //Left Back
      analogWrite(motorW3_IN1, speed1PWM);
      digitalWrite(motorW3_IN2, LOW);

      // Right side motors move in opposite direction

      //Right Front
      digitalWrite(motorW4_IN1, LOW);
      analogWrite(motorW4_IN2, speed1PWM);

      //Right Middle
      digitalWrite(motorW5_IN1, LOW);
      analogWrite(motorW5_IN2, speed1PWM);

      //Right Back
      digitalWrite(motorW6_IN1, LOW);
      analogWrite(motorW6_IN2, speed1PWM);
    }
    else if (ch6 > 1500) {
      //Left Front
      digitalWrite(motorW1_IN1, LOW);
      analogWrite(motorW1_IN2, speed1PWM);

      //Left Middle
      digitalWrite(motorW2_IN1, LOW);
      analogWrite(motorW2_IN2, speed1PWM);

      //Left Back
      digitalWrite(motorW3_IN1, LOW);
      analogWrite(motorW3_IN2, speed1PWM);

      // Right side motors move in opposite direction

      //Right Front
      analogWrite(motorW4_IN1, speed1PWM);
      digitalWrite(motorW4_IN2, LOW);

      //Right Middle
      analogWrite(motorW5_IN1, speed1PWM);
      digitalWrite(motorW5_IN2, LOW);

      //Right Back
      analogWrite(motorW6_IN1, speed1PWM);
      digitalWrite(motorW6_IN2, LOW);
    }
  }
  // Monitor the battery voltage
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.00 / 1023.00) * 3.02; // Convert the reading values from 5v to suitable 12V
  // Send battery voltage value to transmitter
  IBusSensor.loop();
  IBusSensor.setSensorMeasurement(1, voltage * 100);
}

void calculateMotorsSpeed() {
  if (ch0 > 1485 && ch0 < 1515) {     //If there is absolutely no steering, then our rover will continue straight on the path at the speed 's'.
    speed1 = speed2 = speed3 = s;
  }
  else {

    // Outer wheels which are the furthest wheels from the turning point will have max speed
    // Due to the rover geometry, all three outer wheels should rotate almost with the same speed.
    speed1 = s;

    // Inner front and back wheels are closer to the turing point and have lower speeds compared to the outer speeds
    speed2 = s * sqrt(pow(d3, 2) + pow((r - d1), 2)) / (r + d4);

    // Inner middle wheel is closest to the turning point, has the lowest speed
    speed3 = s * (r - d4) / (r + d4);
  }

  // Speed value from 0 to 100% to PWM value from 0 to 255
  speed1PWM = map(round(speed1), 0, 100, 0, 255);
  speed2PWM = map(round(speed2), 0, 100, 0, 255);
  speed3PWM = map(round(speed3), 0, 100, 0, 255);
}

void calculateServoAngle() {
  // Calculate the angle for each servo for the input turning radius "r"
  thetaInnerFront = round((atan((d3 / (r + d1)))) * 180 / PI);
  thetaInnerBack = round((atan((d2 / (r + d1)))) * 180 / PI);
  thetaOuterFront = round((atan((d3 / (r - d1)))) * 180 / PI);
  thetaOuterBack = round((atan((d2 / (r - d1)))) * 180 / PI);

}
