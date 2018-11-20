/*
 * YEAR 8 ROBOT - PROGRAM v2.1
 * ------------------------
 * 
 * This program will drive the robot according to the commands entered.
 * This version uses less code than Program v2.0 because moves repeated lines into functions.
 */


// INTIALISE MOTOR CONTROL UNIT (MCU)
// Motor A
#define ENA 3    // PWM Pin 3 to MCU ENA
#define IN1 2    // Digital Pin 2 to MCU IN1
#define IN2 4    // Digital Pin 4 to MCU IN2

// Motor B
#define ENB 6    // PWM Pin 6 to MCU ENB
#define IN3 5    // Digital Pin 5 to MCU IN3
#define IN4 7    // Digital Pin 7 to MCU IN4

// DEFINE VARIABLES
int motorSpeed = 100;    // value 0 - 255
int time_delay = 1000;

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
  
  delay(2000);          // makes the program wait before continuing onto the next instruction (meassured in milliseconds).
  forwards();
  delay(time_delay);
  backwards();
  delay(time_delay);
  sharp_left();
  delay(time_delay);
  sharp_right();
  delay(time_delay);
  slow_left();
  delay(time_delay);
  slow_right();
  delay(time_delay);
  robot_stop();
  delay(time_delay);
}
