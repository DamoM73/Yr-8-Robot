/*
 * YEAR 8 ROBOT - PROGRAM v3.1
 * ------------------------
 * 
 * This program will make the robot follow a black line on a white background
 * You will need your black value and white value from program v3.0
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

// INTIALISE LINE FOLLOWING SENSORS
#define LEFTSENSOR A0      // Left sensor connected to pin A0 (analogue 0)
#define RIGHTSENSOR A1     // Right sensor connected to pin a1 (analogue 1)

// DEFINE VARIABLES
int motorSpeed = 75;                  // value 0 - 255
int time_delay = 1000;                // in milliseconds
int white = 300;                      // white sensor value from program v3.0
int black = 0;                        // black sensor value from program v3.0
int threshold = (white + black) / 2;  // calculates the threshold point where the robot switches from black to white.

void setup() {
  // SETUP SERIAL MONITOR FOR TROUBLESHOOTING 
  Serial.begin(9600);
  
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
  /*
   * The program first tests the values of the sensors. 
   * If both sensors show white, then the robot goes forward
   * If the left sensor shows black, then the robot does a slow left turn
   * If the right sensor shows black, then the robot does a slow right turn
   * If both sensors show black, then the robot stops
   */

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
