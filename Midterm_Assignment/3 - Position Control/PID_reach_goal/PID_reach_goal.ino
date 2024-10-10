// Motor pins
int enL = 6;       // Left motor enable pin (PWM control)
int inL1 = 8;      // Left motor input 1
int inL2 = 9;      // Left motor input 2
int enR = 7;       // Right motor enable pin (PWM control)
int inR1 = 10;     // Right motor input 1
int inR2 = 11;     // Right motor input 2

int enLA = 18;
int enLB = 19;

int enRA = 3;
int enRB = 2;

volatile int leftEnCount = 0; // # of pulses by the left encoders
volatile int rightEnCount = 0; // # of pulses by the right encoders

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

// PID control constants for distance and angle
// float Kp_distance = 5.0;
// float Ki_distance = 0.01;
// float Kd_distance = 0.1;

float Kp_angle = 5.0;
float Ki_angle = 0.01;
float Kd_angle = 0.2;

// Target position and orientation
float targetX = -1.68;     // meters
float targetY = 0.81;     // meters
float targetTheta = -90; // degrees

// PID variables
// float integral_distance = 0;
// float pre_error_distance = 0;
float integral_angle = 0;
float pre_error_angle = 0;

float currentX = 0.0;
float currentY = 0.0;
float currentTheta = 0.0; // degrees

// Motor speed variables
int baseSpeed = 70;   // Base speed for motors

// Wheel and robot parameters
float wheelRadius = 0.02159; // meters (wheel radius)
float wheelDiameter = wheelRadius * 2;
float wheelDistance = 0.188;    // meters (distance between wheels)
float pulsesPerRevolution = 700; // Encoder pulses per wheel revolution

void setup() {
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(enLA), leftEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enLB), leftEnISRB, RISING);

  attachInterrupt(digitalPinToInterrupt(enRA), rightEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enRB), rightEnISRB, RISING);

  // Motor pin setup
  pinMode(enL, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);

  // Initial motor state
  stopMotors();
}

void loop() {
  // Calculate the Euclidean distance to the target point -> distance error
  float dx = targetX - currentX;
  float dy = targetY - currentY;
  float error_distance = sqrt(dx * dx + dy * dy);


  // Calculate the target angle based on the target position
  float targetAngle = atan2(targetY - currentY, targetX - currentX) * (180.0 / PI);
  
  // Calculate the angle error (difference between current and target angles)
  float error_angle = targetAngle - currentTheta;

  // Normalize angle error to the range [-180, 180] degrees
  if (error_angle > 180) error_angle -= 360;
  if (error_angle < -180) error_angle += 360;

  // Calculate PID outputs for distance
  // integral_distance += error_distance;
  // float derivative_distance = error_distance - pre_error_distance;
  // float output_distance = Kp_distance * error_distance +
  //                         Ki_distance * integral_distance +
  //                         Kd_distance * derivative_distance;
                          
  // pre_error_distance = error_distance;

  // Calculate PID outputs for angle
  integral_angle += error_angle;
  float derivative_angle = error_angle - pre_error_angle;
  float output_angle = Kp_angle * error_angle +
                       Ki_angle * integral_angle +
                       Kd_angle * derivative_angle;

  pre_error_angle = error_angle;

  // // Adjust motor speeds based on PID outputs
  // int leftSpeed = baseSpeed + output_distance - output_angle;
  // int rightSpeed = baseSpeed + output_distance + output_angle;

  int leftSpeed = baseSpeed - output_angle;
  int rightSpeed = baseSpeed + output_angle;

  // Constrain motor speeds to PWM limits
  leftSpeed = constrain(leftSpeed, 50, 180);
  rightSpeed = constrain(rightSpeed, 50, 180);

  // Set motor speeds
  motorLRun(leftSpeed);
  motorRRun(rightSpeed);

  // Update robot state 
  updatePosition();

  // Stop when the robot reaches the target position and orientation
  if ((error_distance < 0.2) && (abs(error_angle) < 20)) {
    stopMotors();
    while (true); // Stay stopped
  }
}


void updatePosition() {

  double leftDistance = (leftEnCount / (double)pulsesPerRevolution) * (PI * wheelDiameter);
  double rightDistance = (rightEnCount / (double)pulsesPerRevolution) * (PI * wheelDiameter);
  
   // Compute robot's change in position and orientation
  double deltaDistance = (leftDistance + rightDistance) / 2.0;
  double deltaTheta = (rightDistance - leftDistance) / wheelDistance;

  // Update robot's global position and orientation
  currentTheta += deltaTheta;
  currentX += deltaDistance * cos(currentTheta * (PI / 180));
  currentY += deltaDistance * sin(currentTheta * (PI / 180));

  // Keep theta in the range [-180, 180]
  if (currentTheta > 180) currentTheta -= 360;
  if (currentTheta < -180) currentTheta += 360;

  // Reset encoder counts for next loop
  leftEnCount = 0;
  rightEnCount = 0;
}

void motorLRun(int speed) {
  if (speed > 0) {
    digitalWrite(inL1, HIGH);
    digitalWrite(inL2, LOW);
  } else {
    digitalWrite(inL1, LOW);
    digitalWrite(inL2, HIGH);
    speed = -speed;
  }
  analogWrite(enL, speed);
}

void motorRRun(int speed) {
  if (speed > 0) {
    digitalWrite(inR1, HIGH);
    digitalWrite(inR2, LOW);
  } else {
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, HIGH);
    speed = -speed;
  }
  analogWrite(enR, speed);
}

void stopMotors() {
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
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
