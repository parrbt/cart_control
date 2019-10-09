/*
   cart_teleop_advanced.ino - Contains main teleop code for cart along with some experimental voltage addressing for steering.

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

  // Print out each value from data */
  Serial.println(throttleVal);
  Serial.println(brakeVal);
  Serial.println(desired);

  if (throttleVal != -1 && brakeVal != -1 && desired != -1) {

    // read in the potentiometer value and map from 0 (full left) to 100 (full right)
    acheived = analogRead(wiper);
    acheived = map(acheived, 100, 330, 0, 100);

    // send commands to handler functions
    setThrottle(throttleVal);
    setBrake(brakeVal);
    calculateSteering(desired, acheived, throttleVal);
  }

}

/* sends a desired voltage to throttle dac */
void setThrottle(int throttleVal) {
  tcaselect(2);
  int val = map(throttleVal, 0, 255, 409, 3645);
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
  int val = map(brakeVal, 0, 255, 409, 3767);
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
void calculateSteering(int desired, int acheived, int throttleVal) {
  int lowerBound = desired - 3;
  int upperBound = desired + 3;

  if (acheived < lowerBound || acheived > upperBound) {

    // apply more steering power if we have low or no throttle
    if (throttleVal <= 15) {
      // determines where to steer given the desired and acheived turn angles
      if (acheived < lowerBound) {
        Serial.print("Steering right:\t\t");
        Serial.println(acheived);
        // right steering
        steer(2.0, 3.0);

      } else if (acheived > upperBound) {
        Serial.print("Steering left:\t\t");
        Serial.println(acheived);
        // left steering
        steer(3.0, 2.0);

      } else {
        Serial.print("Neutral steering:\t\t");
        Serial.println(acheived);
        // neutral steering
        steer(2.5, 2.5);
      }

      // less power needed if we are moving a bit
    } else if (throttleVal <= 90) {
      // determines where to steer given the desired and acheived turn angles
      if (acheived < lowerBound) {
        Serial.print("Steering right:\t\t");
        Serial.println(acheived);
        // right steering
        steer(2.25, 2.74);

      } else if (acheived > upperBound) {
        Serial.print("Steering left:\t\t");
        Serial.println(acheived);
        // left steering
        steer(2.75, 2.25);

      } else {
        Serial.print("Neutral steering:\t\t");
        Serial.println(acheived);
        // neutral steering
        steer(2.5, 2.5);
      }

    }



    // re-read wiper to see if we have reached the desired angle yet
    acheived = analogRead(wiper);
    acheived = map(acheived, 100, 330, 0, 100);
  }
}


/* sends a desired voltage to both left and right steering channels */
void steer(float leftVal, float rightVal) {
  tcaselect(4);
  // check if values are valid - must be between 2.0 & 3.0 inclusive and sum up to 5.0
  if ((leftVal >= 2.0 && leftVal <= 3.0) && (rightVal >= 2.0 && rightVal <= 3.0) && (leftVal + rightVal == 5.0)) {
    int finalLeft = mapf(leftVal, 0, 5, 0, 4095);
    int finalRight = mapf(rightVal, 0, 5, 0, 4095);

    // 0 for ground (0V) 4095 for max voltage (5V)
    leftSteer.setVoltage(finalLeft, false);
    rightSteer.setVoltage(finalRight, false);
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
