#include <Encoder.h>
// read current position, count of rotations, use interrupts ble ble
// basically does the same as the other code

#include <math.h>

// Motor pins
int leftMotorPin1 = 8;
int leftMotorPin2 = 9;

int rightMotorPin1 = 10;
int rightMotorPin2 = 11;

// Encoder pins
int enLA = 18;
int enLB = 19;

int enRA = 2;
int enRB = 3;

const float wheel_distance = 18.796; // cm
const float wheel_radius = 2.159; //cm
const float T = 1.2;   // Time step

// Control gains
const float gamma = 0.2;
const float lamda = 0.25;
const float h = 0.15;

// Current position and orientation
float x = 0.0;
float y = 0.0;
float theta = 0.0;

// Target position
const float x_g = 1.0;
const float y_g = 1.0;
float theta_g = 0.0;

Encoder encoderLeft(encoderLeftPinA, encoderLeftPinB);
Encoder encoderRight(encoderRightPinA, encoderRightPinB);

void setup() {
  Serial.begin(9600);

  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  // Turn off motors - Initial state
	digitalWrite(leftMotorPin1, LOW);
	digitalWrite(leftMotorPin2, LOW);
	digitalWrite(rightMotorPin1, LOW);
	digitalWrite(rightMotorPin2, LOW);
}

void loop() {
  // Calculate errors
  while (abs(x_g - x) > 0.05) {
    float deltaX = x_g - x;
    float deltaY = y_g - y;
    float rho = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
    float phi = atan2(deltaY, deltaX) - theta_g;
    float alpha = atan2(deltaY, deltaX) - theta;

    // calculate errors: -> ???
    float distance_error = sqrt(deltaX * deltaX + deltaY * deltaY);
    float desired_theta = atan2(deltaY, deltaX);
    float heading_error = desired_theta - theta;

    // Adjust heading_error to be within [-pi, pi] ->  ???
    if (heading_error > PI) heading_error -= 2 * PI;
    if (heading_error < -PI) heading_error += 2 * PI;


    float v = gamma * cos(alpha) * rho;
    float w = lamda * alpha + gamma * cos(alpha) * sin(alpha) * (alpha + h * phi) / alpha;
    float vr = v + d * w / 2;
    float vl = v - d * w / 2;

    // Convert velocities to RPM
    float wr = vr / r * 60 / (2 * PI);
    float wl = vl / r * 60 / (2 * PI);

    // Set motor speeds
    set_speedL(wl);
    set_speedR(wr);
    
    delay(T * 1000); // Convert seconds to milliseconds
    
    // Read motor speeds
    float v1 = get_speedL();
    float v2 = get_speedR();
    
    // Update position and orientation
    x = x + (v1 + v2) / 2 * cos(theta) * T;
    y = y + (v1 + v2) / 2 * sin(theta) * T;
    theta = theta + (v2 - v1) / d * T;
    
    // Debugging outputs
    Serial.print("x: "); Serial.print(x);
    Serial.print(" y: "); Serial.print(y);
    Serial.print(" theta: "); Serial.println(theta);
  }

  // Check if goal is reached
  if (distance_error < 0.01 && abs(heading_error) < 0.01) {
    Serial.println("Goal reached!");
    while (true); // Stop the loop
    set_speedL(0);
    set_speedR(0);
  }

  delay(100); // Adjust delay for your control frequency
}

void set_speedL(float speed) {
  // Function to set the speed of the left motor
  // Implement motor control logic
}

void set_speedR(float speed) {
  // Function to set the speed of the right motor
  // Implement motor control logic
}

float get_speedL() {
  // Function to get the speed of the left motor
  // Implement motor speed reading logic
  return 0.0;
}

float get_speedR() {
  // Function to get the speed of the right motor
  // Implement motor speed reading logic
  return 0.0;
}

