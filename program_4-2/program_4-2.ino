/*
 * YEAR 8 ROBOT - PROGRAM v3.1
 * ------------------------
 * 
 * This program will make the robot follow a black line on a white background
 * When it identifies an obsticle in the way, it will stop and move around it
 */

#include <HCSR04.h>                     // Tells the program to use the HCSR04 library

// INTIALISE MOTOR CONTROL UNIT (MCU)
// Motor A
#define ENA 3                           // PWM Pin 3 to MCU ENA
#define IN1 2                           // Digital Pin 2 to MCU IN1
#define IN2 4                           // Digital Pin 4 to MCU IN2

// Motor B
#define ENB 6                           // PWM Pin 6 to MCU ENB
#define IN3 5                           // Digital Pin 5 to MCU IN3
#define IN4 7                           // Digital Pin 7 to MCU IN4

// INITIALISE ULTRASONIC SENSOR
#define TRIG_PIN 13                     // HCSR04 trigger pin connected to pin 13
#define ECHO_PIN 12                     // HCSR04 echo pin connected to pin 12

// INTIALISE LINE FOLLOWING SENSORS
#define LEFTSENSOR A0                   // Left sensor connected to pin A0 (analogue 0)
#define RIGHTSENSOR A1                  // Right sensor connected to pin a1 (analogue 1)

// DEFINE VARIABLES
int motorSpeed = 75;                    // value 0 - 255
int time_delay = 1000;                  // in milliseconds
int white = 300;                        // white sensor value from program v3.0
int black = 0;                          // black sensor value from program v3.0
int threshold = (white + black) / 2;    // calculates the threshold point where the robot switches from black to white.
int min_dist = 5;                       // the minimum distance before stopping (cm)

// START LIBRARY
HCSR04 hc(TRIG_PIN, ECHO_PIN);          // Sends details for the library code to use

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

  // SETUP LINE FOLLOWING SENSORS
  // Set all pins to input
  pinMode(LEFTSENSOR, INPUT);
  pinMode(RIGHTSENSOR, INPUT);
  
  Serial.begin(9600);                   // initlaise serial monintor for troubleshooting

}
// MOTOR COMMANDS
// Right Motor Commands

void right_motor_fwd(){
  // This code should make the right motor drive forwards. If it drives backwards swap the LOW and HIGH values for IN1 and IN2
  digitalWrite(IN1, LOW);               // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN2, HIGH);              // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENA, motorSpeed);         // analogWrite can send values 0 - 255
}

void right_motor_bck(){
  // This code should make the right motor drive backwards. If it drives forwards swap the LOW and HIGH values for IN1 and IN2
  digitalWrite(IN1, HIGH);              // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN2, LOW);               // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENA, motorSpeed);         // analogWrite can send values 0 - 255
}

void right_motor_stop(){
  // This code should make the right motor stop. It does not matter if IN1 & IN2 are LOW or HIGH, just that they are the same.
  digitalWrite(IN1, HIGH);              // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN2, HIGH);              // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENA, motorSpeed);         // analogWrite can send values 0 - 255
}

// Left Motor Commands

void left_motor_fwd(){
  // This code should make the left motor drive forwards. If it drives backwards swap the LOW and HIGH values for IN3 and IN4
  digitalWrite(IN3, HIGH);              // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN4, LOW);               // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENB, motorSpeed);         // analogWrite can send values 0 - 255
}

void left_motor_bck(){
  // This code should make the left motor drive backwards. If it drives frowards swap the LOW and HIGH values for IN3 and IN4
  digitalWrite(IN3, LOW);               // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN4, HIGH);              // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENB, motorSpeed);         // analogWrite can send values 0 - 255
}

void left_motor_stop(){
  // This code should make the left motor stop. It does not matter if IN3 & IN4 are LOW or HIGH, just that they are the same.
  digitalWrite(IN3, LOW);               // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN4, LOW);               // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENB, motorSpeed);         // analogWrite can send values 0 - 255
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
  /*
   * If there is no object closer than min_dist, then robot will line follow.
   * If there is an object closer than min_dist, then the robot will go around
   */

  if (hc.dist() < min_dist){                // testing that for object in the way
    robot_stop();
    sharp_left();
    delay(100);
    forwards();
    delay(100);
    sharp_right();
    delay(100);
    forwards();
    delay(200);
    sharp_right();
    delay(100);
    forwards(100);
    sharp(left);
    delay(100);
    robot_stop();
  }else{
    // GET SENSOR VALUES
    int left = analogRead(LEFTSENSOR);
    int right = analogRead(RIGHTSENSOR);

    // RESPOND TO VALUES
    if (left < threshold && right < threshold){       // both sensors are white
        forwards();
    }
    else if (left > threshold && right < threshold){  // left is black, right is right
        slow_left();
    }
    else if (left < threshold && right > threshold){  // left is white, right is black
        slow_right();
    }
    else if (left > threshold && right > threshold){  // both sensors are black
        robot_stop(); 
    }
  }
}
