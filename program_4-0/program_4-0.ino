/*
 * YEAR 8 ROBOT - PROGRAM v4.0
 * 
 * This program will test the ultrasonic rangefinding sensor to ensure that it is connected correctly.
 * This program uses a library, which is code written by someone else.
 * The library has to be installed in your Arduino IDE before you can use it.
 */

#include <HCSR04.h>       // Tells the program to use the library we installed

// INITIALISE SENSOR
#define TRIG_PIN 13       // HCSR04 trigger pin connected to pin 13
#define ECHO_PIN 12       // HCSR04 echo pin connected to pin 12

// START LIBRARY
HCSR04 hc(TRIG_PIN, ECHO_PIN); // Sends details for the library code to use

void setup() {
  Serial.begin(9600);   // initlaise serial monintor to view readings  
}

void loop() {
  // Take the readings from the sensor and show them on the serial monitor
  Serial.print("Distance: ");           
  Serial.print(hc.dist());         // prints the information provided by the library
  Serial.println("cm.");
  delay(500);                                 
}
