/*
 * YEAR 8 ROBOT - PROGRAM v3.0
 * 
 * This program will test the line following sensors to ensure that they are connected correctly.
 * It will also show you the sensor values for black and white, you will need this for the next part.
 */

// INITIALISE LINE FOLLOWING SENSORS
#define LEFTSENSOR A0      // Left sensor connected to pin A0 (analogue 0)
#define RIGHTSENSOR A1     // Right sensor connected to pin a1 (analogue 1)

void setup() {
  Serial.begin(9600);   // initlaise serial monintor to view readings
  
  // SETUP LINE FOLLOWING SENSORS
  // Set all pins to input
  pinMode(LEFTSENSOR, INPUT);
  pinMode(RIGHTSENSOR, INPUT);
  
}

void loop() {
  // Take the readings from the sensors and show them on the serial monitor
  Serial.print("Left(Y): ");                  // write text to serial monitor
  Serial.print(analogRead(LEFTSENSOR));       // read LEFTSENSOR and write value to serial monitor
  Serial.print(", Right(P): ");               // write text to serial monitor
  Serial.println(analogRead(RIGHTSENSOR));    // read right sensor and write value to serial monitor
  delay(500);                                 // wait half a second
}
