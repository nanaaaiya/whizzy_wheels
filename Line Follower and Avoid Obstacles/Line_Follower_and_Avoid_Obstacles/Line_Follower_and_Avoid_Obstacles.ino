#include <HCSR04.h>

// Define pins for ultrasonic sensors
#define FRONT_TRIGGER_PIN 46
#define FRONT_ECHO_PIN 47
#define LEFT_TRIGGER_PIN 25
#define LEFT_ECHO_PIN 27
#define RIGHT_TRIGGER_PIN 50
#define RIGHT_ECHO_PIN 51

// Left Motor
int enL = 6;
int inL1 = 8;
int inL2 = 9;

// Right motor 
int enR = 7;
int inR1 = 10;
int inR2 = 11;

// For encoder
int enLA = 19;
int enLB = 18;

int enRA = 3;
int enRB = 2;

volatile int leftEnCount = 0; // # of pulses by the left encoders
volatile int rightEnCount = 0; // # of pulses by the right encoders

// Pin configuration for the line-following sensor
int LFOut1 = 32;
int LFOut2 = 34;
int LFOut3 = 36;
int LFOut4 = 38;
int LFOut5 = 40; 


int baseSpeedL = 65; // Base speed for the left motor
int baseSpeedR = 65; // Base speed for the right motor


// PID control variables
// Proportional gain  measures “how far” the robot is away from the line
float Kp = 3; 
// The integral term sums error to determine “how long” 
// the robot has been away from the line
float Ki = 0.01; // Integral gain
// The derivative term assesses “how fast” error in the process is changing
float Kd = 6.5; // Derivative gain

// tune p, d, fuck i

float error = 0;      // Current error
float lastError = 0;  // Previous error
float I = 0;          // Integral term
float D = 0;          // Derivative term
float gain = 0;       // PID gain


// Define distance threshold
const int threshold = 18; // Distance in cm to start avoiding
const int wallDistance = 25; // Ideal distance from the wall while following

bool obstacle;
bool complete_line = false,
int a;

// Ultrasonic sensor objects
HCSR04 sonarFront(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
HCSR04 sonarLeft(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
HCSR04 sonarRight(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);

float distanceLeft;
float distanceFront;
float distanceRight;
void lineFollow();
void avoidObstacle(int distanceLeft, int distanceRight);

void setup() {
  Serial.begin(9600);

  // Set all line following sensor pins as input
  pinMode(LFOut1, INPUT);
  pinMode(LFOut2, INPUT);
  pinMode(LFOut3, INPUT);
  pinMode(LFOut4, INPUT);
  pinMode(LFOut5, INPUT);

  // Set motor pins as outputs
  pinMode(enL, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);

  obstacle = false;
  a = 0;

  // Setup encoder interrupts
  attachInterrupt(digitalPinToInterrupt(enLA), leftEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enLB), leftEnISRB, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), rightEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enRB), rightEnISRB, RISING);

  int sensor1 = digitalRead(LFOut1);
  int sensor2 = digitalRead(LFOut2);
  int sensor3 = digitalRead(LFOut3);
  int sensor4 = digitalRead(LFOut4);
  int sensor5 = digitalRead(LFOut5);

  // Turn off motors - Initial state
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
}


void loop() {  
  // Get distances from the front, left, and right ultrasonic sensors
  distanceFront = sonarFront.dist();
  distanceLeft = sonarLeft.dist();
  distanceRight = sonarRight.dist();

  // // Debugging print for sensor readings
  // Serial.print("Front: ");
  // Serial.print(distanceFront);
  // Serial.print(" cm, Left: ");
  // Serial.print(distanceLeft);
  // Serial.print(" cm, Right: ");
  // Serial.print(distanceRight);
  // Serial.println(" cm");

     // Calculate the error based on sensor readings
  int sensor1 = digitalRead(LFOut1);
  int sensor2 = digitalRead(LFOut2);
  int sensor3 = digitalRead(LFOut3);
  int sensor4 = digitalRead(LFOut4);
  int sensor5 = digitalRead(LFOut5); // closest the the tag

  if ((sensor1 == 1||sensor2 == 1||sensor3 == 1||sensor4 == 1||sensor5 == 1) && (obstacle == false) && (complete_line == false)){
    lineFollow();
    // Obstacle avoidance
    if (distanceFront > 1 && distanceFront < threshold) {  // Obstacle in front
      obstacle = true;
    } 
  else ejiowjeoifweiojr 
  }
  if (obstacle == true){ 
    avoidObstacle(distanceLeft, distanceRight);
  //  obstacle = false;
  }
}


void stop() {
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
}

void lineFollow() {
   // Calculate the error based on sensor readings
  int sensor1 = digitalRead(LFOut1);
  int sensor2 = digitalRead(LFOut2);
  int sensor3 = digitalRead(LFOut3);
  int sensor4 = digitalRead(LFOut4);
  int sensor5 = digitalRead(LFOut5); // closest the the tag

  error = calculateError(sensor1, sensor2, sensor3, sensor4, sensor5);

  // PID calculation
  I = I + error; // Integral term
  D = error - lastError; // Derivative term
  gain = Kp * error + Ki * I + Kd * D; // Total PID gain

  // Motor speed adjustment
  int speedL = baseSpeedL - gain;
  int speedR = baseSpeedR + gain;

  Serial.print("Left speed:");
  Serial.println(speedL);

  Serial.print("Right speed:");
  Serial.println(speedR);

  // Constrain motor speeds within the allowed range
  speedL = constrain(speedL, 50, 255);
  speedR = constrain(speedR, 50, 255);

  // Set motor speeds
	analogWrite(enR, speedR);
  analogWrite(enL, speedL);

	// Turn on motor A & B
	digitalWrite(inL1, HIGH);
	digitalWrite(inL2, LOW);
	digitalWrite(inR1, LOW);
	digitalWrite(inR2, HIGH);

  // Update last error
  lastError = error;

  // delay(10); // Small delay for stability
}
// Function to calculate the error based on the sensor input
float calculateError(int s1, int s2, int s3, int s4, int s5) {
  // 0 0 1 0 0
  if (s3 == HIGH && s1 == LOW && s2 == LOW && s4 == LOW && s5 == LOW ) return 0;    // Center is on the line, no error
  // 0 1 1 0 0 
  else if (s2 == HIGH && s3 == HIGH && s1 == LOW && s4 == LOW && s5 == LOW) return 1; 
  // 0 0 1 1 0
  else if (s3 == HIGH && s4 == HIGH && s1 == LOW && s2 == LOW && s5 == LOW) return -1;
  // 1 1 0 0 0
  else if (s1 == HIGH && s2 == HIGH && s3 == LOW && s4 == LOW && s5 == LOW) return 3; 
  // 0 0 0 1 1
  else if (s4 == HIGH && s5 == HIGH && s1 == LOW && s2 == LOW && s3 == LOW) return -3;
  // 0 1 0 0 0
  else if (s2 == HIGH && s1 == LOW && s3 == LOW && s4 == LOW && s5 == LOW) return 2;   // Slightly to the left
  // 0 0 0 1 0
  else if (s4 == HIGH && s1 == LOW && s2 == LOW && s3 == LOW && s5 == LOW) return -2;    // Slightly to the right
  // 1 0 0 0 0
  else if (s1 == HIGH && s2 == LOW && s3 == LOW && s4 == LOW && s5 == LOW) return 4;   // Far to the left
  // 0 0 0 0 1
  else if (s5 == HIGH && s1 == LOW && s2 == LOW && s3 == LOW && s4 == LOW) return -4;    // Far to the right
  else 
  return 0;                 // Default case (no line detected)
}

void turnLeft() {
  // Left motor stops, right motor moves forward
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);  
  digitalWrite(inR1, LOW);  
  digitalWrite(inR2, HIGH);
  analogWrite(enL, 200);   // Stop left motor
  analogWrite(enR, 200); // Move right motor
}

void moveForward() {
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);  
  digitalWrite(inR2, HIGH); 
  analogWrite(enL, 70); // Speed for left motor (0-255)
  analogWrite(enR, 70); // Speed for right motor (0-255)
}

void turnRight() {
  // Right motor stops, left motor moves forward
  digitalWrite(inL1, HIGH);  // Move left motor forward
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, HIGH);   // Stop right motor
  digitalWrite(inR2, LOW);
  analogWrite(enL, 200); // Move left motor
  analogWrite(enR, 200);   // Stop right motor
}

void followWallLeft() {  
    // Calculate the error based on sensor readings
  int sensor1 = digitalRead(LFOut1);
  int sensor2 = digitalRead(LFOut2);
  int sensor3 = digitalRead(LFOut3);
  int sensor4 = digitalRead(LFOut4);
  int sensor5 = digitalRead(LFOut5); // closest the the tag

  if (sensor1 == 1||sensor2 == 1||sensor3 == 1||sensor4 == 1||sensor5 == 1) {
    stop();
    delay(1000);
    moveForward();
    delay(500);
    turnRight();
    delay(200);
    lineFollow();
    obstacle = false;
    a = 0;
    complete_line = false,

  }
  else{
    if (distanceLeft > 0 && distanceLeft < wallDistance) { 
      // Too far from wall, move closer
      turnRightSlight();
    } else { 
      // Too close to the wall, move away
      turnLeftSlight();
    } 
  }
}

void followWallRight() { 
    // Calculate the error based on sensor readings
  int sensor1 = digitalRead(LFOut1);
  int sensor2 = digitalRead(LFOut2);
  int sensor3 = digitalRead(LFOut3);
  int sensor4 = digitalRead(LFOut4);
  int sensor5 = digitalRead(LFOut5); // closest the the tag

  if (sensor1 == 1||sensor2 == 1||sensor3 == 1||sensor4 == 1||sensor5 == 1) {
    stop();
    delay(1000);
    moveForward();
    delay(500);
    turnLeft();
    delay(200);
    lineFollow();
    obstacle = false;
    a = 0;
    complete_line = false,

  }
  else{
    if (distanceRight > 0 && distanceRight < wallDistance) { 
      // Too far from wall, move closer
      turnLeftSlight();
    } else { 
      // Too close to the wall, move away
      turnRightSlight();
    } 
  } 
}

void turnLeftSlight() {
  // Slight left turn to correct path
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);
  analogWrite(enL, 50);
  analogWrite(enR, 100);
}

void turnRightSlight() {
  // Slight right turn to correct path
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);
  analogWrite(enL, 100);
  analogWrite(enR, 50);
}

void avoidObstacle(int distanceLeft, int distanceRight) {
  if (distanceLeft > distanceRight) {
    // Turn left and start wall-following
    if (a == 0){
      stop();
      delay(500);
      turnLeft();
      delay(250);
      // stop();
      // delay(20000000);
      a = 1;
    }
  }
  else {
    if (a == 0){ // Turn right and start wall-following
      stop();
      delay(500); 
      turnRight();
      delay(250); // Turn for a short duration
      // stop();
      // delay(20000000);
      a = 2;
    }
  }

  if (a == 1) {
    followWallRight();
  }
  else if (a == 2){
    followWallLeft();
  }
} 


void travel_w_distance(int dist) { 
  float circumference = 2.159 * 2 * 3.14; // cm
  float rotations = dist/ circumference;
  int pulses = rotations * 700;

  stop();
  delay(5000);

  // Move forward
  digitalWrite(inL1, HIGH);
	digitalWrite(inL2, LOW);
	digitalWrite(inR1, LOW);
	digitalWrite(inR2, HIGH);
  // Set motor speeds
	analogWrite(enR, 50);
  analogWrite(enL, 50);

  // Wait until the required pulses are reached
  while(leftEnCount < pulses && rightEnCount < pulses) {
  }

  stop();
  // Serial.println(pulses);
  // Serial.print("Left Encoder Count: ");
  // Serial.println(leftEnCount);
  // Serial.print("Right Encoder Count: ");
  // Serial.println(rightEnCount);

  // delay(5000); 

}


void leftEnISRA() {
  leftEnCount++;
}

void leftEnISRB() {
  leftEnCount++;
}
void rightEnISRA() {
  rightEnCount++;
}

void rightEnISRB() {
  rightEnCount++;
}
