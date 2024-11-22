#include <TimerOne.h> 

// Define encoder pins
#define LEFT_ENCODER_A 3
#define LEFT_ENCODER_B 2
#define RIGHT_ENCODER_A 19
#define RIGHT_ENCODER_B 18

// Motor Pins
int enL = 6;
int inL1 = 8;
int inL2 = 9;
int enR = 7;
int inR1 = 10;
int inR2 = 11;

// Define wheel and robot parameters
const float WHEEL_RADIUS = 0.02155; // Wheel radius in meters
const float WHEEL_BASE = 0.18796;    // Distance between wheels in meters
const int TICKS_PER_REV = 700;  // Encoder ticks per wheel revolution
// const float PI = 3.14;

// Variables for encoder counts
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// Variables to store robot state
float x = 0.0, y = 0.0, theta = 0.0; // Position and orientation

void setup() {
  Serial.begin(9600);
  // Initialize TimerOne to call the `toggleLED` function every 1 second (1000000 microseconds)
  Timer1.initialize(250000);  // 1 second in microseconds
  Timer1.attachInterrupt(updatePos);  // Attach the interrupt function (toggleLED)

  // Set encoder pins as inputs
  pinMode(LEFT_ENCODER_A, INPUT);
  pinMode(LEFT_ENCODER_B, INPUT);
  pinMode(RIGHT_ENCODER_A, INPUT);
  pinMode(RIGHT_ENCODER_B, INPUT);

  // Attach interrupt service routines for encoders
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), updateRightEncoder, CHANGE);
}

void loop() {
  goForward(); // Move forward 
  delay(3000);

  turnRight(); //turn right
  delay(500);

  goForward();
  delay(3000);

  turnRight();
  delay(500);

  goForward();
  delay(3000);

  turnRight(); 
  delay(500); 

  goForward(); 
  delay(2000); 

  stop();
  while(1) {};
  // Send data back over Serial
  

  // Add a delay to match the desired update rate (e.g., 10 Hz)
  delay(1000);
}


void updatePos() {
    static long prevLeftTicks = 0, prevRightTicks = 0;

  // Disable interrupts temporarily to read shared variables
  noInterrupts();
  long leftTicksCopy = leftTicks;
  long rightTicksCopy = rightTicks;
  interrupts();

  // Calculate tick differences
  long deltaLeft = leftTicksCopy - prevLeftTicks;
  long deltaRight = rightTicksCopy - prevRightTicks;

  // Update previous ticks
  prevLeftTicks = leftTicksCopy;
  prevRightTicks = rightTicksCopy;

    // Convert ticks to distance
  float leftDistance = 2 * 3.14159 * WHEEL_RADIUS * deltaLeft / TICKS_PER_REV;
  float rightDistance = 2 * 3.14159 * WHEEL_RADIUS * deltaRight / TICKS_PER_REV;

  // Compute linear and angular displacement
  float deltaDistance = (leftDistance + rightDistance) / 2.0;
  float deltaTheta = (rightDistance - leftDistance) / WHEEL_BASE;

  // Update position and orientation
  theta += deltaTheta;
  x += deltaDistance * cos(theta);
  y += deltaDistance * sin(theta);

    //Serial.print("X: ");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.println(theta);

}

// Interrupt Service Routine for left encoder
void updateLeftEncoder() {
  static bool lastA = LOW, lastB = LOW;
  bool newA = digitalRead(LEFT_ENCODER_A);
  bool newB = digitalRead(LEFT_ENCODER_B);

  if (newA != lastA) {
    if (newA == newB)
      leftTicks++;
    else
      leftTicks--;
  }
  lastA = newA;
  lastB = newB;
}

// Interrupt Service Routine for right encoder
void updateRightEncoder() {
  static bool lastA = LOW, lastB = LOW;
  bool newA = digitalRead(RIGHT_ENCODER_A);
  bool newB = digitalRead(RIGHT_ENCODER_B);

  if (newA != lastA) {
    if (newA == newB)
      rightTicks++;
    else
      rightTicks--;
  }
  lastA = newA;
  lastB = newB;
}

void goForward() {
  analogWrite(enR, 90);
  analogWrite(enL, 100);

  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);
}

void stop() {
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
}

void turnRight() {
  analogWrite(enR, 85); // left stop, right moves forward 
  analogWrite(enL, 85);    // Full speed on left motor

  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);
}

void turnLeft() {
  analogWrite(enR, 85);    // Full speed on right motor
  analogWrite(enL, 85); // Reduce left motor speed

  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH); 
}

