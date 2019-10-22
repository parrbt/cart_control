/*
   cart_teleop.ino - Contains the main arduino code for controlling our autonomous golf cart.

      Author: Brandon Parr
      Version: 1.0
*/

#include <Adafruit_MCP4725.h>
#include <Wire.h>
#include "utility/twi.h"

#define TCAADDR 0x70

Adafruit_MCP4725 throttle;    /* port 2a */
Adafruit_MCP4725 brake;       /* port 2b */
Adafruit_MCP4725 leftSteer;   /* port 4a */
Adafruit_MCP4725 rightSteer;  /* port 4b */

int throttleRelay = 9;        /* pin 9  */
int brakeRelay = 8;           /* pin 8  */
int wiper = A6;               /* pin A3 */

/* bounds for throttle mapping */
int lowerThrottleBounds = 409;
int upperThrottleBounds = 3645;

/* bounds for brake mapping */
int lowerBrakeBounds = 409;
int upperBrakeBounds = 3767;

/* bounds for brake mapping */
int lowerSteerBounds = 0;
int upperSteerBounds = 4095;

/* high, low, and neutral steer values */
float lowSteer = 2.3;
float highSteer = 2.7;
float neutralSteer = 2.5;

/* magic numbers */
const int magicStart = 42;
const int magicEnd = 21;

/* Main program setup */
void setup() {

  // set up relays
  pinMode(throttleRelay, OUTPUT);
  pinMode(brakeRelay, OUTPUT);

  // set up serial
  Serial.begin(9600);
  while (!Serial) {
    delay(15);
  }

  /* Sets up dacs connected to multiplexer  */
  /*     a is 0x62, b is 0x63               */
  /*     Port 2a: throttle                  */
  /*     Port 2b: brake                     */
  /*     Port 4a: left steer                */
  /*     Port 4b: right steer               */
  tcaselect(2);
  throttle.begin(0x62);
  brake.begin(0x63);
  Serial.println("Throttle and brake dacs started!");

  tcaselect(4);
  leftSteer.begin(0x62);
  rightSteer.begin(0x63);
  Serial.println("left and right steering dacs started!");
  Serial.println("STARTING");
}

/* Main program loop */
void loop() {
  delay(5);

  readCommands();
}

void readCommands() {

  // values read in from serial
  int throttleVal = -1;
  int brakeVal = -1;
  int desired = -1;

  // potentiometer value read from analog on board
  int acheived = -1;


  // Read in throttle, brake, and steering data
  throttleVal = Serial.read();
  brakeVal = Serial.read();
  desired = Serial.read();


  if (throttleVal != -1 && brakeVal != -1 && desired != -1) {

    // read in the potentiometer value and map from 0 (full left) to 100 (full right)
    acheived = analogRead(wiper);
    acheived = map(acheived, 100, 330, 0, 100);

    // send commands to handler functions
    setThrottle(throttleVal);
    setBrake(brakeVal);
    calculateSteering(desired, acheived);

    // Print out each value from data */
    Serial.println(throttleVal);
    Serial.println(brakeVal);
    Serial.println(desired);
  }

}

/* sends a desired voltage to throttle dac */
void setThrottle(int throttleVal) {
  tcaselect(2);
  int val = map(throttleVal, 0, 255, lowerThrottleBounds, upperThrottleBounds);
  throttle.setVoltage(val, false);

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
  tcaselect(2);
  int val = map(brakeVal, 0, 255, lowerBrakeBounds, upperBrakeBounds);
  brake.setVoltage(val, false);

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
      steer(lowSteer, highSteer);
    } else if (acheived > upperBound) {
      Serial.print("Steering left:\t\t");
      Serial.println(acheived);
      // left steering
      steer(highSteer, lowSteer);
    }
  } else {
    Serial.print("Neutral steering:\t\t");
    Serial.println(acheived);
    // neutral steering
    steer(neutralSteer, neutralSteer);
  }
}


/* sends a desired voltage to both left and right steering channels */
void steer(float leftVal, float rightVal) {
  tcaselect(4);
  // check if values are valid - must be between 2.0 & 3.0 inclusive and sum up to 5.0
  if ((leftVal >= lowSteer && leftVal <= highSteer) && (rightVal >= lowSteer && rightVal <= highSteer) && (leftVal + rightVal == 5.0)) {
    int finalLeft = mapf(leftVal, 0, 5, lowerSteerBounds, upperSteerBounds);
    int finalRight = mapf(rightVal, 0, 5, lowerSteerBounds, upperSteerBounds);

    // 0 for ground (0V) 4095 for max voltage (5V)
    leftSteer.setVoltage(finalLeft, false);
    rightSteer.setVoltage(finalRight, false);
  } else {
    Serial.println("INVALID VOLTAGE. . .");
    Serial.println("\tPlease select a voltage between 2.3 - 2.7 volts");
    Serial.println("\tLeft and right steering must sum to 5.0 volts");
  }
}

/* map function for a float value */
float mapf(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

/* Sets up a given port on a multiplexer */
void tcaselect(uint8_t i) {
  if (i > 7) {
    Serial.println("INVALID tcaselect. . .");
    Serial.println("\tPlease select a value between 0 and 7");
    return;
  }

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
