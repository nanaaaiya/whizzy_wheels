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

// Right Motor
int enR = 7;
int inR1 = 10;
int inR2 = 11;

// Define distance threshold
const int threshold = 25; // Distance in cm to start avoiding
const int wallDistance = 25; // Ideal distance from the wall while following

bool obstacle;
int a;

// Ultrasonic sensor objects
HCSR04 sonarFront(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
HCSR04 sonarLeft(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
HCSR04 sonarRight(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);

float distanceLeft;
float distanceFront;
float distanceRight;

void setup() {
  Serial.begin(9600);

  // Set motor pins as outputs
  pinMode(enL, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);

  obstacle = false;
  a = 0;
}

void loop() {  
  // Get distances from the front, left, and right ultrasonic sensors
  distanceFront = sonarFront.dist();
  distanceLeft = sonarLeft.dist();
  distanceRight = sonarRight.dist();

  // Debugging print for sensor readings
  Serial.print("Front: ");
  Serial.print(distanceFront);
  Serial.print(" cm, Left: ");
  Serial.print(distanceLeft);
  Serial.print(" cm, Right: ");
  Serial.print(distanceRight);
  Serial.println(" cm");

  if (obstacle == false){
    moveForward();
    // Obstacle avoidance logic
    if (distanceFront > 1 && distanceFront < threshold) {  // Obstacle in front
      obstacle = true;
    } 
  }
  else {
    avoidObstacle(distanceLeft, distanceRight);
  }
}

void moveForward() {
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);  
  digitalWrite(inR2, HIGH); 
  analogWrite(enL, 70); // Speed for left motor (0-255)
  analogWrite(enR, 70); // Speed for right motor (0-255)
}

void stop() {
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
}

void avoidObstacle(int distanceLeft, int distanceRight) {
  if (distanceLeft > distanceRight) {
    // Turn left and start wall-following
    if (a == 0){
      stop();
      delay(500); // Pause
      turnLeft();
      delay(250);
      a = 1;
    }
    // delay(500); // Turn for a short duration
  } else{
    // Turn right and start wall-following
      if (a == 0){
        stop();
        delay(500); // Pause
        turnRight();
        delay(250); // Turn for a short duration
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


void turnLeft() {
  // Left motor stops, right motor moves forward
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);  
  digitalWrite(inR1, LOW);  
  digitalWrite(inR2, HIGH);
  analogWrite(enL, 200);   // Stop left motor
  analogWrite(enR, 200); // Move right motor
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
  if (distanceLeft >= 1 && distanceLeft < wallDistance) { 
    // Too far from wall, move closer
    turnRightSlight();
  } else { 
    // Too close to the wall, move away
    turnLeftSlight();
  } 
}

void followWallRight() {  
  if (distanceRight >= 1 && distanceRight < wallDistance) { 
    // Too far from wall, move closer
    turnLeftSlight();
  } else { 
    // Too close to the wall, move away
    turnRightSlight();
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