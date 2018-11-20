/*
 * YEAR 8 ROBOT - PROGRAM v4.0
 * 
 * This program will allow the robot to drive around and avoid running into things
 * When the robot senses that it is near an object it will stop, reverse and turn in another direction
 */

#include <HCSR04.h>       // Tells the program to use the HCSR04 library

// INTIALISE MOTOR CONTROL UNIT (MCU)
// Motor A
#define ENA 3    // PWM Pin 3 to MCU ENA
#define IN1 2    // Digital Pin 2 to MCU IN1
#define IN2 4    // Digital Pin 4 to MCU IN2

// Motor B
#define ENB 6    // PWM Pin 6 to MCU ENB
#define IN3 5    // Digital Pin 5 to MCU IN3
#define IN4 7    // Digital Pin 7 to MCU IN4


// INITIALISE ULTRASONIC SENSOR
#define TRIG_PIN 13       // HCSR04 trigger pin connected to pin 13
#define ECHO_PIN 12       // HCSR04 echo pin connected to pin 12

// DEFINE VARIABLES
int motorSpeed = 100;     // value 0 - 255
int time_delay = 1000;
int min_dist = 5;         // the minimum distance before stopping (cm)

// START LIBRARY
HCSR04 hc(TRIG_PIN, ECHO_PIN); // Sends details for the library code to use

void setup() {
  // SETUP MOTOR CONTROL UNIT
  // Set all pins to output
  
  // Motor A
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  // Motor B
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(9600);   // initlaise serial monintor for troubleshooting
}

// MOTOR COMMANDS
// Right Motor Commands

void right_motor_fwd(){
  // This code should make the right motor drive forwards. If it drives backwards swap the LOW and HIGH values for IN1 and IN2
  digitalWrite(IN1, LOW);         // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN2, HIGH);        // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENA, motorSpeed);   // analogWrite can send values 0 - 255
}

void right_motor_bck(){
  // This code should make the right motor drive backwards. If it drives forwards swap the LOW and HIGH values for IN1 and IN2
  digitalWrite(IN1, HIGH);         // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN2, LOW);        // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENA, motorSpeed);   // analogWrite can send values 0 - 255
}

void right_motor_stop(){
  // This code should make the right motor stop. It does not matter if IN1 & IN2 are LOW or HIGH, just that they are the same.
  digitalWrite(IN1, HIGH);         // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN2, HIGH);         // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENA, motorSpeed);    // analogWrite can send values 0 - 255
}

// Left Motor Commands

void left_motor_fwd(){
  // This code should make the left motor drive forwards. If it drives backwards swap the LOW and HIGH values for IN3 and IN4
  digitalWrite(IN3, HIGH);        // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN4, LOW);         // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENB, motorSpeed);   // analogWrite can send values 0 - 255
}

void left_motor_bck(){
  // This code should make the left motor drive backwards. If it drives frowards swap the LOW and HIGH values for IN3 and IN4
  digitalWrite(IN3, LOW);        // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN4, HIGH);         // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENB, motorSpeed);   // analogWrite can send values 0 - 255
}

void left_motor_stop(){
  // This code should make the left motor stop. It does not matter if IN3 & IN4 are LOW or HIGH, just that they are the same.
  digitalWrite(IN3, LOW);        // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN4, LOW);         // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENB, motorSpeed);   // analogWrite can send values 0 - 255
}

// ROBOT COMMANDS

void forwards(){
  right_motor_fwd();
  left_motor_fwd();
}

void backwards(){
  right_motor_bck();
  left_motor_bck();
}

void sharp_left(){
  right_motor_fwd();
  left_motor_bck();
}

void sharp_right(){
  right_motor_bck();
  left_motor_fwd();
}

void slow_left(){
  right_motor_fwd();
  left_motor_stop(); 
}

void slow_right(){
  right_motor_stop();
  left_motor_fwd();
}

void robot_stop(){
  right_motor_stop();
  left_motor_stop();
}


void loop() {
  
  /* Robot will move forward until it get too close to an object. 
   * It will then stop, reverse and turn left for a random time. 
   * It will then drive forward
   */

  delay(time_delay);            // waits for the HCSR04 to take a reading
   
  if (hc.dist() > min_dist){    // checks if there is an object is too close
    forwards();                 // if no object is in the way then drive
  }else {                       // otherwise...
    robot_stop();               // stop
    backwards();                // reverse...
    delay(random(250,1000));    // for a random amount of time
    sharp_left();               // turn left...
    delay(random(250,1000));    // for a random amount of time
  }                                 
}
