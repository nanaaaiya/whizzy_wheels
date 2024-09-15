// Motor control pins
// Left Motor
int enL = 7;
int inL1 = 8;
int inL2 = 9;

// Right Motor
int enR = 6;
int inR1 = 10;
int inR2 = 11;

// For encoder
int enLA = 18;
int enLB = 19;

int enRA = 2;
int enRB = 3;

// Constants
const float d = 0.189;    // Distance between wheels (meters)
const float r = 0.0216;   // Radius of the wheels (meters)
const float T = 0.05;     // Time step (seconds)

// PID parameters (adjust these to tune)
// 1) Kp (Proportional Gain): determines how aggressively 
//the controller reacts to the current error
// Increase gradually until you see the system starting to 
// oscillate or become unstable.

// 2) Ki (Integral Gain): Addresses the accumulated error over time
// helps eliminate steady-state errors but 
// can lead to oscillations if set too high.
// Increase until you notice that the 
// steady-state error (offset from the desired value) is corrected

// 3) Kd (Derivative Gain): Predicts future error based on 
// its rate of change, helping to dampen oscillations and improve stability.
// Increase to reduce the overshoot and oscillations by Kp or Ki.

float Kp_v = 1.5, Ki_v = 0.0, Kd_v = 0.5; // For linear velocity control
float Kp_w = 1.0, Ki_w = 0.0, Kd_w = 0.5; // For angular velocity control

// Goal position and orientation (1,1,0)
const float goalX = 1.0;  // Goal X position (meters)
const float goalY = 1.0;  // Goal Y position (meters)
const float goalTheta = 0.0; // Goal orientation (radians)

// Encoder constants
const int encoderCountsPerRevolution = 700;
const float wheelDiameter = r * 2;
const float wheelCircumference = wheelDiameter * M_PI;

// Robot state variables
float x = 0.0;     // Current X position (meters)
float y = 0.0;     // Current Y position (meters)
float theta = 0.0; // Current orientation (radians)

// Encoder counts
volatile int leftEnCount = 0;
volatile int rightEnCount = 0;

// PID control variables
float error_v, error_w;
float integral_v = 0, integral_w = 0;
float previous_error_v = 0, previous_error_w = 0;

void setup() {
  Serial.begin(9600);

  // Setup encoder interrupts
  attachInterrupt(digitalPinToInterrupt(enLA), leftEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enLB), leftEnISRB, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), rightEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enRB), rightEnISRB, RISING);

  // Set all motor control pins to outputs
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);

  // Turn off motors initially
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
}

void loop() {
  // Calculate position error
  float deltaX = goalX - x;
  float deltaY = goalY - y;
  float rho = sqrt(deltaX * deltaX + deltaY * deltaY); // Distance to goal
  float phi = atan2(deltaY, deltaX) - goalTheta;       // Desired direction
  float alpha = atan2(deltaY, deltaX) - theta;         // Heading error

  // ???????
  // Desired velocities (linear and angular)
  float v = Kp_v * rho;  // Simple P controller for linear velocity
  float w = Kp_w * alpha; // Simple P controller for angular velocity

  // PID control for v and w
  // Linear velocity PID
  error_v = rho; // is it correct?
  integral_v += error_v * T; // accumulates the error over time
  float derivative_v = (error_v - previous_error_v) / T;
  v = Kp_v * error_v + Ki_v * integral_v + Kd_v * derivative_v;
  previous_error_v = error_v;

  // Angular velocity PID
  error_w = alpha;
  integral_w += error_w * T;
  float derivative_w = (error_w - previous_error_w) / T;
  w = Kp_w * error_w + Ki_w * integral_w + Kd_w * derivative_w;
  previous_error_w = error_w;

  // feedback co sai j k ta :)?

  // Compute wheel velocities //
  float vr = v + d * w / 2.0;
  float vl = v - d * w / 2.0;
  float wr = vr * 60.0 / (2.0 * M_PI * r); // Right wheel RPM
  float wl = vl * 60.0 / (2.0 * M_PI * r); // Left wheel RPM

  // Set wheel speeds
  set_speedL(wl);
  set_speedR(wr);

  // Update robot position using odometry
  updatePosition();

  // Print debugging information
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Theta: ");
  Serial.println(theta);

  // Check if goal is reached
  if (fabs(goalX - x) <= 0.05 && fabs(goalY - y) <= 0.05) {
    stop(); // Stop the robot
    while (1); // Stop further execution
  }

  delay(T * 1000); // Wait for the next cycle
}

void updatePosition() {
  // Update robot position and orientation based on wheel encoder counts
  float deltaL = (leftEnCount / (float)encoderCountsPerRevolution) * wheelCircumference;
  float deltaR = (rightEnCount / (float)encoderCountsPerRevolution) * wheelCircumference;
  float deltaTheta = (deltaR - deltaL) / d;
  
  // Update x, y, and theta
  float deltaDistance = (deltaL + deltaR) / 2.0;
  x += deltaDistance * cos(theta);
  y += deltaDistance * sin(theta);
  theta += deltaTheta;

  // Reset encoder counts
  leftEnCount = 0;
  rightEnCount = 0;
}

// Function to set the speed of the left wheel
void set_speedL(float speed) {
  int pwmValue = map(speed, 0, 480, 0, 255);
  analogWrite(enL, pwmValue);
}

// Function to set the speed of the right wheel
void set_speedR(float speed) {
  int pwmValue = map(speed, 0, 480, 0, 255);
  analogWrite(enR, pwmValue);
}

// Encoder ISR for left wheel
void leftEnISRA() {
  leftEnCount++;
}

void leftEnISRB() {
  leftEnCount++;
}

// Encoder ISR for right wheel
void rightEnISRA() {
  rightEnCount++;
}

void rightEnISRB() {
  rightEnCount++;
}

// Stop the motors
void stop() {
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
}
