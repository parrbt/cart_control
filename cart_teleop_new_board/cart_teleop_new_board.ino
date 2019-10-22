/*
<<<<<<< HEAD
   cart_teleop_new_board.ino - Contains the main arduino code for controlling our autonomous golf cart with non-dac board.
=======
   cart_teleop.ino - Contains the main arduino code for controlling our autonomous golf cart.
>>>>>>> 364151c1e18dd3dbbf603327690a636c4c460996

      Author: Brandon Parr
      Version: 1.0
*/

<<<<<<< HEAD
=======
#include <Adafruit_MCP4725.h>
#include <Wire.h>
#include "utility/twi.h"

#define TCAADDR 0x70

>>>>>>> 364151c1e18dd3dbbf603327690a636c4c460996
int throttle = 3;         /* pin 3 */
int brake = 5;            /* pin 5 */
int leftSteer = 6;        /* pin 6 */
int throttleRelay = 7;    /* pin 7 */
<<<<<<< HEAD
int brakeRelay = 8;       /* pin 8 */
int rightSteer = 9;       /* pin 9 */

int wiper = A0;           /* pin A0 */

/* bounds for throttle mapping */
int lowerThrottleBounds = 25;
int upperThrottleBounds = 226;

/* bounds for brake mapping */
int lowerBrakeBounds = 25;
int upperBrakeBounds = 234;

/* bounds for brake mapping */
int lowerSteerBounds = 0;
int upperSteerBounds = 100;

/* high, low, and neutral steer values */
float lowSteer = 2.3;
float highSteer = 2.7;
float neutralSteer = 2.5;

/* magic numbers */
const int magicStart = 42;
const int magicEnd = 21;
=======
int brakeRelay = 8;       /* pin 8  */
int rightSteer = 9;       /* pin 9 */

int wiper = A0;           /* pin A3 */
>>>>>>> 364151c1e18dd3dbbf603327690a636c4c460996

/* Main program setup */
void setup() {

  // set up serial
  Serial.begin(9600);
  while (!Serial) {
    delay(15);
  }

  Serial.println("SETTING UP PINS");

  // set up relays
  pinMode(throttleRelay, OUTPUT);
  pinMode(brakeRelay, OUTPUT);

  // set up outputs
  pinMode(throttle, OUTPUT);
  pinMode(brake, OUTPUT);
  pinMode(leftSteer, OUTPUT);
  pinMode(rightSteer, OUTPUT);

  // set up wiper
  pinMode(wiper, INPUT);

  delay(500);
  Serial.println(".");
  delay(500);
  Serial.println(".");
  delay(500);
  Serial.println(".");
  delay(500);
  Serial.println("STARTING");
}

/* Main program loop */
void loop() {
  delay(5);
  readCommands();
}

void readCommands() {

  // values read in from serial
<<<<<<< HEAD
  int firstByte = -1;
  int secondByte = -1;
  int throttleVal = -1;
  int brakeVal = -1;
  int desired = -1;
  
  // potentiometer value read from analog on board
  int acheived = -1;

  // check to see if you have gotten the magic numbers.
  while (firstByte != magicStart || secondByte != magicEnd) {
    firstByte = Serial.read();
    secondByte = Serial.read();
    Serial.println(firstByte);
    Serial.println(secondByte);
  }
=======
  int throttleVal = -1;
  int brakeVal = -1;
  int desired = -1;

  // potentiometer value read from analog on board
  int acheived = -1;

>>>>>>> 364151c1e18dd3dbbf603327690a636c4c460996

  // Read in throttle, brake, and steering data
  throttleVal = Serial.read();
  brakeVal = Serial.read();
  desired = Serial.read();

<<<<<<< HEAD
=======

>>>>>>> 364151c1e18dd3dbbf603327690a636c4c460996
  if (throttleVal != -1 && brakeVal != -1 && desired != -1) {

    // read in the potentiometer value and map from 0 (full left) to 100 (full right)
    acheived = analogRead(wiper);
    acheived = map(acheived, 100, 330, 0, 100);

    // send commands to handler functions
    setThrottle(throttleVal);
    setBrake(brakeVal);
    calculateSteering(desired, acheived);
  }
<<<<<<< HEAD
=======

>>>>>>> 364151c1e18dd3dbbf603327690a636c4c460996
}

/* sends a desired voltage to throttle dac */
void setThrottle(int throttleVal) {
<<<<<<< HEAD
  int val = map(throttleVal, 0, 255, lowerThrottleBounds, upperThrottleBounds);
=======
  int val = map(throttleVal, 0, 255, 25, 226);
>>>>>>> 364151c1e18dd3dbbf603327690a636c4c460996
  analogWrite(throttle, val);

  Serial.print("Throttle set:\t\t");
  Serial.println(throttleVal);
  if (throttleVal == 0) {
    digitalWrite(throttleRelay, LOW);
  } else {
    digitalWrite(throttleRelay, HIGH);
  }
}

/* sends a desired voltage to brake dac */
void setBrake(int brakeVal) {
<<<<<<< HEAD
  int val = map(brakeVal, 0, 255, lowerBrakeBounds, upperBrakeBounds);
=======
  int val = map(brakeVal, 0, 255, 25, 234);
>>>>>>> 364151c1e18dd3dbbf603327690a636c4c460996
  analogWrite(brake, val);

  Serial.println("Brake set:\t\t");
  Serial.println(brakeVal);
  if (brakeVal == 255) {
    digitalWrite(brakeRelay, LOW);
  } else {
    digitalWrite(brakeRelay, HIGH);
  }
}

/* calculates left and right steering voltages given the desired angle and acheived angle */
void calculateSteering(int desired, int acheived) {
  int lowerBound = desired - 3;
  int upperBound = desired + 3;

  if (acheived < lowerBound || acheived > upperBound) {

    // determines where to steer given the desired and acheived turn angles
    if (acheived < lowerBound) {
      Serial.print("Steering right:\t\t");
      Serial.println(acheived);
      // right steering
<<<<<<< HEAD
      steer(lowSteer, highSteer);
=======
      steer(2.0, 3.0);

>>>>>>> 364151c1e18dd3dbbf603327690a636c4c460996
    } else if (acheived > upperBound) {
      Serial.print("Steering left:\t\t");
      Serial.println(acheived);
      // left steering
<<<<<<< HEAD
      steer(highSteer, lowSteer);
    }
=======
      steer(3.0, 2.0);

    }
    
>>>>>>> 364151c1e18dd3dbbf603327690a636c4c460996
  } else {
    Serial.print("Neutral steering:\t\t");
    Serial.println(acheived);
    // neutral steering
<<<<<<< HEAD
    steer(neutralSteer, neutralSteer);
=======
    steer(2.5, 2.5);
>>>>>>> 364151c1e18dd3dbbf603327690a636c4c460996
  }
}


/* sends a desired voltage to both left and right steering channels */
void steer(float leftVal, float rightVal) {
  // check if values are valid - must be between 2.0 & 3.0 inclusive and sum up to 5.0
  if ((leftVal >= 2.0 && leftVal <= 3.0) && (rightVal >= 2.0 && rightVal <= 3.0) && (leftVal + rightVal == 5.0)) {
    int finalLeft = mapf(leftVal, 0, 5, 0, 255);
    int finalRight = mapf(rightVal, 0, 5, 0, 255);

    // 0 for ground (0V) 4095 for max voltage (5V)
<<<<<<< HEAD
    analogWrite(leftSteer, finalLeft);
    analogWrite(rightSteer, finalRight);
=======
    analogWrite(leftSteer, finalLeft + 30);
    analogWrite(rightSteer, finalRight + 30);
>>>>>>> 364151c1e18dd3dbbf603327690a636c4c460996

  } else {
    Serial.println("INVALID VOLTAGE. . .");
    Serial.println("\tPlease select a voltage between 2.0 - 3.0 volts");
    Serial.println("\tLeft and right steering must sum to 5.0 volts");
  }
}

/* map function for a float value */
float mapf(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
