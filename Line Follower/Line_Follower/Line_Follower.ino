// Motor Pins
int enL = 6;       // Left motor enable pin (PWM control)
int inL1 = 8;      // Left motor input 1
int inL2 = 9;      // Left motor input 2
int enR = 7;       // Right motor enable pin (PWM control)
int inR1 = 10;     // Right motor input 1
int inR2 = 11;     // Right motor input 2

// Pin configuration for the line-following sensor
int LFOut1 = 32;
int LFOut2 = 34;
int LFOut3 = 36;
int LFOut4 = 38;
int LFOut5 = 40; 


int baseSpeedL = 62; // Base speed for the left motor
int baseSpeedR = 62; // Base speed for the right motor


// PID control variables
// Proportional gain  measures “how far” the robot is away from the line
float Kp = 3; 
// The integral term sums error to determine “how long” 
// the robot has been away from the line
float Ki = 0.01; // Integral gain
// The derivative term assesses “how fast” error in the process is changing
float Kd = 6.5; // Derivative gain

// tune p, d
// tune basespeed, kd if turning k on dinh

float error = 0;      // Current error
float lastError = 0;  // Previous error
float I = 0;          // Integral term
float D = 0;          // Derivative term
float gain = 0;       // PID gain

void setup() {
  Serial.begin(9600);

  // // Setup encoder interrupts
  // attachInterrupt(digitalPinToInterrupt(enLA), leftEnISRA, RISING);
  // attachInterrupt(digitalPinToInterrupt(enLB), leftEnISRB, RISING);
  // attachInterrupt(digitalPinToInterrupt(enRA), rightEnISRA, RISING);
  // attachInterrupt(digitalPinToInterrupt(enRB), rightEnISRB, RISING);

  // Set all the motor control pins to outputs
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);


  // Set all line following sensor pins as input
  pinMode(LFOut1, INPUT);
  pinMode(LFOut2, INPUT);
  pinMode(LFOut3, INPUT);
  pinMode(LFOut4, INPUT);
  pinMode(LFOut5, INPUT);

  // Turn off motors - Initial state
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
}

void loop() {
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

// // Encoder ISR for left wheel
// void leftEnISRA() {
//   leftEnCount++;
// }

// void leftEnISRB() {
//   leftEnCount++;
// }

// // Encoder ISR for right wheel
// void rightEnISRA() {
//   rightEnCount++;
// }

// void rightEnISRB() {
//   rightEnCount++;
// }

// Stop the motors
void stop() {
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
}


void travel_w_distance(int dist) { 
  // Summarize: tính chu vi bánh xe -> từ đó tìm đc rotation của bánh xe và cuối cùng
  // là pulse cần thiết của motor để quay 1 vòng. cho điều kiện 2 bánh chạy đủ pulse
  // thì dừng
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
  Serial.println(pulses);
  Serial.print("Left Encoder Count: ");
  Serial.println(leftEnCount);
  Serial.print("Right Encoder Count: ");
  Serial.println(rightEnCount);

  delay(5000); 

}