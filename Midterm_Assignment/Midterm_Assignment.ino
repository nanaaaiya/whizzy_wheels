// Left Motor
int enL = 6;
int inL1 = 8;
int inL2 = 9;

// Right motor -> hhh
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

int pulses_per_rev = 700; 
const float wheel_radius = 2.159; //cm
const float wheel_diameter = wheel_radius * 2;
const float wheel_distance = 18.796; // cm

float rpmLeft = 0;
float rpmRight = 0;

void setup()
{
  Serial.begin(9600);

  // Setup interrupt 
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

  digitalWrite(inR1, LOW);
	digitalWrite(inR2, LOW);
	digitalWrite(inL1, LOW);
	digitalWrite(inL2, LOW);

}

void loop() {
  goForward(240);

}

void goForward(int speed) {
  // Reset encoder counter
  rightEnCount = 0;
  leftEnCount = 0;

	// For PWM maximum possible values are 0 to 255
	analogWrite(enR, speed);
  analogWrite(enL, speed);

	// Turn on motor A & B
	digitalWrite(inL1, HIGH);
	digitalWrite(inL2, LOW);
	digitalWrite(inR1, LOW);
	digitalWrite(inR2, HIGH);
}

void stop() {
	// Turn off motors 
	digitalWrite(inR1, LOW);
	digitalWrite(inR2, LOW);
	digitalWrite(inL1, LOW);
	digitalWrite(inL2, LOW);
}

void travel_w_distance(int dist) { 
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
  stop();
  delay(2000);  
  
  rpmLeft = float((float(leftEnCount) * 60.0) / pulses_per_rev);
  rpmRight = float((float(rightEnCount) * 60.0) / pulses_per_rev); 

  // why when left/ right count is 2500, rpm only 23, 43 :D?
  // this function produces incorrect outputs

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