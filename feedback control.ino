// Motor control pins
// Left Motor
int enL = 6;
int inL1 = 8;
int inL2 = 9;

// Right motor
int enR = 7;
int inR1 = 10;
int inR2 = 11;

// For encoder
int enLA = 2;
int enLB = 3;

int enRA = 18;
int enRB = 19;

unsigned long lastTime = 0;

// Control constants
const float d = 0.189;    // Distance between wheels (meters)
const float r = 0.0216; // Radius of the wheels (meters)
const float T = 0.2;      // Sampling time (seconds)

// increase -> higher speed
const float gamma = 0.7;  // Linear control gain
// controls how sharply the robot turns to align with its target
const float lamda = 3; // Angular control gain
// reduce deviation errors when the robot’s path isn't aligned well
const float h = 1.25;     // Offset in angle calculation

// Encoder constantss
int pulses_per_rev = 700; 
const float wheel_diameter = r*2; // Diameter of the wheel in meters
//const float M_PI = 3.14159265358979323846;
const float wheel_circumference = wheel_diameter * M_PI ; // 

// const float x_g = -1.68;  // Goal X position (meters)
// const float y_g = 0.81;  // Goal Y position (meters)
// const float theta_g = -90; // Goal orientation (radians)

const float x_g = 1;  // Goal X position (meters)
const float y_g = 1;  // Goal Y position (meters)
const float theta_g = 0; // Goal orientation (radians)

// Robot state variables
float x = 0.0;     // Current X position (meters)
float y = 0.0;     // Current Y position (meters)
float theta = 0.0; // Current orientation (radians)

// Encoder counts and speed control
volatile int leftEnCount = 0;
volatile int rightEnCount = 0;
// const int K = 30;  // Adjustment factor for speed control

long prevTime = 0;

void setup() {
  Serial.begin(9600);

  // Setup encoder interrupts
  attachInterrupt(digitalPinToInterrupt(enLA), leftEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enLB), leftEnISRB, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), rightEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enRB), rightEnISRB, RISING);

  // Set all the motor control pins to outputs
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);

}

void loop() {
  // Calculate control inputs
  float deltaX = x_g - x;
  float deltaY = y_g - y;

  // distance error
  float rho = sqrt(deltaX * deltaX + deltaY * deltaY);
  // angle between the robot’s current heading (orientation) 
  // and the direction from the robot to the goal.
  float phi = atan2(deltaY, deltaX) - theta_g;
  // difference between the angle to the goal and the desired heading at the goal
  float alpha = atan2(deltaY, deltaX) - theta;

  // linear velocity
  float v = gamma * cos(alpha) * rho;
  // angular velocity
  float w = lamda * alpha + gamma * cos(alpha) * sin(alpha) * (alpha + h * phi) / alpha;

  //  Serial.print("v:");
  // Serial.println(v);
  // Serial.print("w:");
  // Serial.println(w); 

  float vr = (2*v + d * w) / 2.0; // v: m/s, w: rad/s or rpm below
  float vl = (2*v - d * w) / 2.0; 
 
  // linear velo to angular velo in Radians per Second: v/r
  // rad/s to RPM: revolution per sec = w/2pi -> rpm: x60
  float wr = vr * 60.0 / (2.0 * r * 3.1412); // Angular velocity in RPM
  float wl = vl * 60.0 / (2.0 * r * 3.1412);

  // Set wheel speeds: convert rpm to 0->255
  set_speedL(wl);
  set_speedR(wr);

  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH); 

  // how to make sure it runs in 1.2 secs
  // Wait for the time step
  delay(T * 1000); // Convert seconds to milliseconds
      
  long currentTime = millis();

  if (currentTime - prevTime >= T * 1000) {
    // float v1 = get_speedL();
    // float v2 = get_speedR();

    float v1 = vl;
    float v2 = vr;

    x += (v1 + v2) / 2.0 * cos(theta) * T;
    y += (v1 + v2) / 2.0 * sin(theta) * T;
    theta += (v2 - v1) / d * T;

    prevTime= currentTime;  // Update lastUpdate to the current time
  }

  // Print debugging information
  Serial.print("X: ");
  Serial.print(x);
  Serial.print("--");
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print("--");
  Serial.print(" Theta: ");
  Serial.println(theta);
  // Serial.print(" Left Speed: ");
  // Serial.println(v1);
  // Serial.print(" Right Speed: ");
  // Serial.println(v2);
  // Serial.println();

  Serial.println("=================");
  Serial.println();

  // Check if goal is reached
  if (fabs(x_g - x) <= 0.1 && fabs(y_g - y) <= 0.1) {
    stop();
    Serial.println("Goal reached!");
    while (1); // Stop further execution
  }
}

// Function to set the speed of the left wheel
void set_speedL(float speed) {
  Serial.print("Left Speed:");

  Serial.println(speed);

  // create a function to control the velo to improve accuracy: + offset
  int pwmValue = speed/300 * 255; // wL is in RPM, map it to 0-255  

  pwmValue = constrain(pwmValue, 100, 245); // Ensure the PWM value is within the valid range
  analogWrite(enL, pwmValue + 10);
}

// Function to set the speed of the right wheel
void set_speedR(float speed) {
  Serial.print("Right Speed:");

  Serial.println(speed);

  int pwmValue = speed/300 * 255; 
  pwmValue = constrain(pwmValue, 100, 245); 

   analogWrite(enR, pwmValue + 10);
}

// Function to get the current speed of the left wheel
float get_speedL() {
  // Convert encoder counts to speed in m/s
  float countsPerSecond = leftEnCount / T; // Counts per second
  float speedRPM = (countsPerSecond * 60) / pulses_per_rev; // RPM
  float speedMetersPerSecond = speedRPM * (wheel_circumference / 60); // m/s
  leftEnCount = 0; // Reset count after reading
  return speedMetersPerSecond;
}

// Function to get the current speed of the right wheel
float get_speedR() {
  // Convert encoder counts to speed in m/s
  float countsPerSecond = rightEnCount / T; // Counts per second
  float speedRPM = (countsPerSecond * 60) / pulses_per_rev; // RPM
  float speedMetersPerSecond = speedRPM * (wheel_circumference / 60); // m/s
  rightEnCount = 0; // Reset count after reading
  // Serial.print("pulses:");
  // Serial.println(countsPerSecond);
  //   Serial.print("rpm:");
  //     Serial.println(speedRPM);

  return speedMetersPerSecond;
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