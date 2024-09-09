// Mega2560
// external interrupt int.0    int.1    int.2   int.3   int.4   int.5            
// pin                  2         3      21      20      19      18

// Left Motor
int enL = 7;
int inL1 = 8;
int inL2 = 9;

// Right motor
int enR = 6;
int inR1 = 10;
int inR2 = 11;

// For encoder
int enLA = 18;
int enLB = 19;

int enRA = 2;
int enRB = 3;

// volatile keyword is used to tell the compiler that the value of  
// the variable declared using volatile may change at any time

volatile int leftEnCount = 0; // # of pulses by the left encoders
volatile int rightEnCount = 0; // # of pulses by the right encoders

int pulses_per_rev = 700; 
const float wheel_radius = 2.159; //cm
const float wheel_diameter = wheel_radius * 2;
const float circle_radius = 100; // cm
const float wheel_distance = 18.796; // cm

float rpmLeft = 0;
float rpmRight = 0;

// A constant used for adjusting the speed difference between 
// the two motors to maintain straight motion or smooth turns. -> 
const int K = 30;  //adjust K for smooth response

void setup()
{
  Serial.begin(9600);

  // Setup interrupt 

  //  whenever it detects a rising edge on pin enLA (the eft motor's 
  // encoder channel A), immediately execute the leftEnISRA() function.
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
  

  
  //  turnLeft();
  // goForward(30);
  // stop();
  // delay(5000);


  // spin_one_round();

  // test computing rpm
  // unsigned long startTime = millis();
  // goForward(100);
  
  // // Wait for exactly 1 second (1000 milliseconds)
  // while (millis() - startTime < 1000) {
  //   // Keep moving forward
  // }
  // compute_rpm();
  
  // travel_w_distance(100);


  // test circle with 2 meters
  circle_2m(100);

  // delay(10000);  // Drive for 10 seconds 

  // stop();  // Stop the robot
  // delay(5000);  // Wait for 5 seconds

}

void circle_2m (int speed) {

  // Summarize vòng tròn 2m: Tính bán kính vòng tròn bánh trái (phía trong, gần tâm hình tròn) và bánh phải (bên ngoài), 
  // từ đó có ratio để tính đc speed của từng bánh

  // Summarize nửa vòng tròn 2m (do c recycle lại code bài 2m luôn): giống như trên, nhưng tính thêm số pulse cần đi
  // như bài 2 (bài travel 1m á), thỏa mãn thì dừng
  float radius_inner = circle_radius - (wheel_distance / 2.0);  // cm
  float radius_outer = circle_radius + (wheel_distance / 2.0);  // cm

  // Calculate the speed ratio
  float speed_ratio = radius_outer / radius_inner;

  // Set the speed for the motors
  int inner_speed = speed;
  int outer_speed = speed * speed_ratio;

  // Set motor speeds and directions for circular motion
  analogWrite(enL, inner_speed);  // Left motor (inner wheel)
  analogWrite(enR, outer_speed);  // Right motor (outer wheel)

  // Go forward in a circle
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);  // Left motor forward
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);   // Right motor forward


  // half a circle

  float circle_circumference = 3.14 * 200; // Half of a circle with diameter 200 cm (2 meters)
  float rotations = (circle_circumference/2) / (2 * 3.14 * wheel_radius);
  int pulses = rotations * pulses_per_rev;
  
  // Wait until both wheels complete the required number of pulses
  while (leftEnCount < pulses && rightEnCount <pulses) {
    // Continue moving until the target is reached
  }
  stop();
}

void goForward(int speed) {
  // Reset encoder counter
  rightEnCount = 0;
  leftEnCount = 0;

	// For PWM maximum possible values are 0 to 255
	analogWrite(enR, speed);

  // adjusts the speed of the left motor to match the right motor
  // int motor_L_speed = speed + K*(rightEnCount-leftEnCount);  
  // analogWrite(enL, motor_L_speed);


  analogWrite(enL, speed);

	// Turn on motor A & B
	digitalWrite(inL1, LOW);
	digitalWrite(inL2, HIGH);
	digitalWrite(inR1, HIGH);
	digitalWrite(inR2, LOW);

}

void stop() {
	// Turn off motors 
	digitalWrite(inR1, LOW);
	digitalWrite(inR2, LOW);
	digitalWrite(inL1, LOW);
	digitalWrite(inL2, LOW);
}


void turnRight() {
  // Reset encoder counter
  rightEnCount = 0;
  leftEnCount = 0;

  int speed = 100;
  const int turnWeight = 2;
	analogWrite(enR, speed);

  int motor_L_speed = turnWeight*speed + K*(turnWeight*rightEnCount-leftEnCount);  
  analogWrite(enL, motor_L_speed);

	// Turn on motor A & B
	digitalWrite(inL1, LOW);
	digitalWrite(inL2, HIGH);
	digitalWrite(inR1, HIGH);
	digitalWrite(inR2, LOW);  
}

void turnLeft() {
  // Reset encoder counter
  rightEnCount = 0;
  leftEnCount = 0;

  int speed = 100;
  const int turnWeight = 2;
	analogWrite(enL, speed);

  int motor_R_speed = turnWeight*speed + K*(turnWeight*leftEnCount-rightEnCount);  
  analogWrite(enR, motor_R_speed);

	digitalWrite(inL1, LOW);
	digitalWrite(inL2, HIGH);
	digitalWrite(inR1, HIGH);
	digitalWrite(inR2, LOW);  
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

void spin_one_round() {
  // Summarize: tính số purses per revolution (tích của xung của encoder với số xung của gear),
  // 2 bánh chạy đủ pulses thì dừng
  
  goForward(100);

  //  if speed is too small (under 100), robot cant run on the ground

  // Wait until the motors have spun one full revolution
  while(leftEnCount < pulses_per_rev && rightEnCount < pulses_per_rev) {
    // Continue moving until target is reached
  }

  // Stop the motors
  stop();

  Serial.print("Left Encoder Count: ");
  Serial.println(leftEnCount);
  Serial.print("Right Encoder Count: ");
  Serial.println(rightEnCount);

  delay(5000);
}

void travel_w_distance(int dist) { 
  // Summarize: tính chu vi bánh xe -> từ đó tìm đc rotation của bánh xe và cuối cùng
  // là pulse cần thiết của motor để quay 1 vòng. cho điều kiện 2 bánh chạy đủ pulse
  // thì dừng
  float circumference = wheel_radius * 2 * 3.14; // cm
  float rotations = dist/ circumference;
  int pulses = rotations * pulses_per_rev;

  stop();
  delay(5000);

  // Move forward
  goForward(100); 

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


void compute_rpm() {
  //  Summarize: như công thức bên dưới :))) 
  // tính số count mỗi s, nhân lên 60 (1ph = 60s) rồi chia cho số pulse 1 revolution để
  // tìm ra rpm từng bánh
  stop();
  delay(2000);  
  
  rpmLeft = (leftEnCount * 60) / pulses_per_rev;
  rpmRight = (rightEnCount * 60) / pulses_per_rev; 

  // why when left/ right count is 2500, rpm only 23, 43 :D?

  Serial.print("Left Encoder Count: ");
  Serial.println(leftEnCount);
  Serial.print("Right Encoder Count: ");
  Serial.println(rightEnCount);
  Serial.print("Left Motor RPM: ");
  Serial.println(rpmLeft);
  Serial.print("Right Motor RPM: ");
  Serial.println(rpmRight);

  // Reset pulse counts for the next measurement
  leftEnCount = 0;
  rightEnCount = 0;
}

