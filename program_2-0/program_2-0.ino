/*
 * YEAR 8 ROBOT - PROGRAM v2.0
 * ------------------------
 * 
 * This program will drive the robot according to the commands entered.
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

void forwards(){
  // RIGHT MOTOR 
  // This code should make the right motor drive forwards. If it drives backwards swap the LOW and HIGH values for IN1 and IN2
  digitalWrite(IN1, LOW);         // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN2, HIGH);        // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENA, motorSpeed);   // analogWrite can send values 0 - 255

  // LEFT MOTOR
  // This code should make the left motor drive forwards. If it drives backwards swap the LOW and HIGH values for IN3 and IN4
  digitalWrite(IN3, HIGH);        // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN4, LOW);         // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENB, motorSpeed);   // analogWrite can send values 0 - 255
}

void backwards(){
  // RIGHT MOTOR
  // This code should make the right motor drive backwards. If it drives forwards swap the LOW and HIGH values for IN1 and IN2
  digitalWrite(IN1, HIGH);         // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN2, LOW);        // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENA, motorSpeed);   // analogWrite can send values 0 - 255

  // LEFT MOTOR 
  // This code should make the left motor drive backwards. If it drives frowards swap the LOW and HIGH values for IN3 and IN4
  digitalWrite(IN3, LOW);        // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN4, HIGH);         // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENB, motorSpeed);   // analogWrite can send values 0 - 255
}

void sharp_left(){
  // RIGHT MOTOR 
  // This code should make the right motor drive forwards. If it drives backwards swap the LOW and HIGH values for IN1 and IN2
  digitalWrite(IN1, LOW);         // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN2, HIGH);        // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENA, motorSpeed);   // analogWrite can send values 0 - 255

  // LEFT MOTOR 
  // This code should make the left motor drive backwards. If it drives frowards swap the LOW and HIGH values for IN3 and IN4
  digitalWrite(IN3, LOW);        // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN4, HIGH);         // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENB, motorSpeed);   // analogWrite can send values 0 - 255
}

void sharp_right(){
  // RIGHT MOTOR
  // This code should make the right motor drive backwards. If it drives forwards swap the LOW and HIGH values for IN1 and IN2
  digitalWrite(IN1, HIGH);         // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN2, LOW);        // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENA, motorSpeed);   // analogWrite can send values 0 - 255
  
  // LEFT MOTOR
  // This code should make the left motor drive forwards. If it drives backwards swap the LOW and HIGH values for IN3 and IN4
  digitalWrite(IN3, HIGH);        // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN4, LOW);         // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENB, motorSpeed);   // analogWrite can send values 0 - 255
}

void slow_left(){
  // RIGHT MOTOR 
  // This code should make the right motor drive forwards. If it drives backwards swap the LOW and HIGH values for IN1 and IN2
  digitalWrite(IN1, LOW);         // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN2, HIGH);        // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENA, motorSpeed);   // analogWrite can send values 0 - 255

  // LEFT MOTOR
  // This code should make the left motor stop. It does not matter if IN3 & IN4 are LOW or HIGH, just that they are the same.
  digitalWrite(IN3, LOW);        // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN4, LOW);         // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENB, motorSpeed);   // analogWrite can send values 0 - 255
}

void slow_right(){
  // RIGHT MOTOR 
  // This code should make the right motor stop. It does not matter if IN1 & IN2 are LOW or HIGH, just that they are the same.
  digitalWrite(IN1, HIGH);         // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN2, HIGH);        // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENA, motorSpeed);   // analogWrite can send values 0 - 255
  
  // LEFT MOTOR
  // This code should make the left motor drive forwards. If it drives backwards swap the LOW and HIGH values for IN3 and IN4
  digitalWrite(IN3, HIGH);        // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN4, LOW);         // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENB, motorSpeed);   // analogWrite can send values 0 - 255
}

void robot_stop(){
  // RIGHT MOTOR 
  // This code should make the right motor stop. It does not matter if IN1 & IN2 are LOW or HIGH, just that they are the same.
  digitalWrite(IN1, HIGH);         // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN2, HIGH);        // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENA, motorSpeed);   // analogWrite can send values 0 - 255

  // LEFT MOTOR
  // This code should make the left motor stop. It does not matter if IN3 & IN4 are LOW or HIGH, just that they are the same.
  digitalWrite(IN3, LOW);        // digitalWrite can only send values 0(LOW) or 1(HIGH)
  digitalWrite(IN4, LOW);         // digitalWrite can only send values 0(LOW) or 1(HIGH)
  analogWrite(ENB, motorSpeed);   // analogWrite can send values 0 - 255
}

void loop() {
  /*
   * Too choose a command remove the '//' in front of it.
   */
  //forwards();
  //backwards();
  //sharp_left();
  //sharp_right();
  //slow_left();
  //slow_right();
  
}
