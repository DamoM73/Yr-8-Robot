/*
 * YEAR 8 ROBOT - PROGRAM v1.0
 * ------------------------
 * 
 * This program should make the robot drive straight forward.
 */


// INTIALISE MOTOR CONTROL UNIT (MCU)
// Right Motor (A)
#define ENA 3    // PWM Pin 3 to MCU ENA
#define IN1 2    // Digital Pin 2 to MCU IN1
#define IN2 4    // Digital Pin 4 to MCU IN2

// Left Motor (B)
#define ENB 6    // PWM Pin 6 to MCU ENB
#define IN3 5    // Digital Pin 5 to MCU IN3
#define IN4 7    // Digital Pin 7 to MCU IN4

// DEFINE VARIABLES
int motorSpeed = 100;    // value 0 - 255

void setup() {
  // SETUP MOTOR CONTROL UNIT
  // Set all pins to output
  
  // Right Motor
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  // Left Motor
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
}

void loop() {
  // RIGHT MOTOR FORWARDS
  // This code should make the right motor drive forwards. If it drives backwards swap the LOW and HIGH values for IN1 and IN2
  digitalWrite(IN1, LOW);         // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN2, HIGH);        // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENA, motorSpeed);   // analogWrite can send values 0 - 255

  // LEFT MOTOR FORWARDS
  // This code should make the left motor drive forwards. If it drives backwards swap the LOW and HIGH values for IN3 and IN4
  digitalWrite(IN3, HIGH);        // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN4, LOW);         // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENB, motorSpeed);   // analogWrite can send values 0 - 255
}
