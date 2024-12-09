// Mega2560
// external interrupt int.0    int.1    int.2   int.3   int.4   int.5            
// pin                  2         3      21      20      19      18

#include <TimerOne.h>

#define PI 3.1415926535897932384626433832795
float d = 0.172; // wheel distance
float r = 0.0225; // wheel radius
float samplingTime = 0.2; //0.5;  // sampling time
const int ENCODER_RESOLUTION = 700;
float M_PER_REV = 2*PI*r;

// Left Motor
int enR = 7;
int inR1 = 9;
int inR2 = 8;

// Right motor
int enL = 5;
int inL1 = 10;
int inL2 = 11;

// For encoder
int enLA = 2;
int enLB = 3;

int enRA = 20;
int enRB = 21;

volatile int leftEnCount = 0;
volatile int rightEnCount = 0;
float vR, vL;

// PID constants
float KpL = 100;  // Proportional gain  //50
// float KiL = 220;  // Integral gain 
// float KdL = 30;  // Derivative gain
float KiL = 260;  // Integral gain 70
float KdL = 33;  // Derivative gain 5

// PID constants
float KpR = 100;  // Proportional gain 50
// float KiR = 300;  // Integral gain
// float KdR = 30;  // Derivative gain
float KiR = 130;  // Integral gain 70
float KdR = 33;  // Derivative gain 5

float set_vL = 0, set_vR = 0;
float err_vL = 0, err_vR = 0, pre_err_vL = 0, pre_err_vR = 0;
float integralL = 0, integralR = 0, derivativeR = 0, derivativeL = 0;
float controlOutputL = 0, controlOutputR = 0;

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
	
	// Turn off motors - Initial state
	digitalWrite(inR1, LOW);
	digitalWrite(inR2, LOW);
	digitalWrite(inL1, LOW);
	digitalWrite(inL2, LOW);

  // Initialize TimerOne to call the toggleLED function every 1 second (1000000 microseconds)
  // Note: This timer will use pin 12 so do not use it for other function 
  Timer1.initialize(1000000*samplingTime); // 1,000,000 microseconds = 1 second
  Timer1.attachInterrupt(VelCtrlTimer); // Attach the interrupt to the calculate velocity 

}

void loop() {
//  positionControl(1,1,0);
  // goForward(100);
//  set_speedL_PID(70);
//  delay(10000);
  //  Serial.print("Right ");
  //  Serial.println(rightEnCount);
  //  Serial.print("Left ");
  //  Serial.println(leftEnCount);
  //  delay(200);
   


  
  // Print velocities to Serial Monitor
  Serial.println("Set to 0.1");
  set_vL = 0.25;
  set_vR = 0.25;

  delay(5000);
   Serial.println("Set to -0.1");
  set_vL = -0.25;
  set_vR = -0.25;
  delay(5000);

}


void VelCtrlTimer() {
  // Calculate wheel velocities in m/s
  // v = (leftEncoderCount/ENCODER_RESOLUTION)  * M_PER_REV / samplingTime
  vR = (float(rightEnCount)/ENCODER_RESOLUTION)*M_PER_REV/samplingTime;
  vL = (float(leftEnCount)/ENCODER_RESOLUTION)*M_PER_REV/samplingTime;

  // Reset encoder counts for the next calculation
  leftEnCount = 0;
  rightEnCount = 0;

/*
  if (set_vL < 0) {
    vL = -vL;
  }

  if (set_vR < 0) {
    vR = -vR;
  }
*/

  // PID calculations
  err_vL = set_vL - vL;
  err_vR = set_vR - vR;
  
  integralR += err_vR * samplingTime;
  derivativeR = (err_vR - pre_err_vR) / samplingTime;
  controlOutputR = KpR * err_vR + KiR * integralR + KdR * derivativeR;
  pre_err_vR = err_vR;

  integralL += err_vL * samplingTime;
  derivativeL = (err_vL - pre_err_vL) / samplingTime;
  controlOutputL = KpL * err_vL + KiL * integralL + KdL * derivativeL;
  pre_err_vL = err_vL;

  // Set the speed = 0 if the set value = 0
  if (set_vL == 0) {
    controlOutputL = 0;
  }

  if (set_vR == 0) {
    controlOutputR = 0;
  }

  setMotorSpeedR((int)controlOutputR);
  setMotorSpeedL((int)controlOutputL);

  Serial.print("vL: ");
  Serial.print(vL);
  Serial.println(" m/s");

  Serial.print("vR: ");
  Serial.print(vR);
  Serial.println(" m/s");


  // Serial.print("P: ");
  // Serial.print(KpR * err_vR);
  // Serial.print(" - I: ");
  // Serial.print(KiR * integralR);
  // Serial.print(" - D: ");
  // Serial.print(KdR * derivativeR);

  // Serial.print(" - All: ");
  // Serial.print((int)controlOutputR);
  // Serial.print(" - vR: ");
  // Serial.println(vR);


}

void setMotorSpeedL(int speed) {

  // Stop
  if (speed == 0) {
    digitalWrite(inL1, LOW);
    digitalWrite(inL2, LOW);   
  }

  // set motor direction
  if (speed > 0) {
    digitalWrite(inL1, LOW);
    digitalWrite(inL2, HIGH);
  }
  
  if (speed < 0) {
    digitalWrite(inL1, HIGH);
    digitalWrite(inL2, LOW);
  }
  
  // Set motor speed
  speed = abs(speed);

  if (speed > 255) {
    speed = 255;
  }
  if (speed < 40) {
    speed = 40;
  }
  
  analogWrite(enL, speed);
}

void setMotorSpeedR(int speed) {

  // Stop
  if (speed == 0) {
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, LOW);   
  }

  // set motor direction
  if (speed > 0) {
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, HIGH);
  }
  
  if (speed < 0) {
    digitalWrite(inR1, HIGH);
    digitalWrite(inR2, LOW);
  }
  
  // Set motor speed
  speed = abs(speed);
  if (speed > 255) {
    speed = 255;
  }
  if (speed < 40) {
    speed = 40;
  }

  analogWrite(enR, speed);
}

/*
void positionControl(float x_g,float y_g,float theta_g) {
  float x = 0;
  float y = 0;
  float theta = 0;
   
  
  // Control parameter
  float gamma = 0.2;
  float lamda = 0.25;
  float h = 0.15;

  while (abs(x_g - x) > 0.1) {
    float  deltaX = x_g - x;
    float  deltaY = y_g - y;
    float rho = sqrt(pow(deltaX,2) + pow(deltaY,2));
    float  phi = atan2(deltaY,deltaX) - theta_g;
    float  alpha = atan2(deltaY,deltaX) - theta;
      
    float  v = gamma*cos(alpha)*rho;
    float  w = lamda*alpha + gamma*cos(alpha)*sin(alpha)*(alpha + h*phi)/alpha;
    float  vr = v + d*w/2;
    float  vl = v - d*w/2;
    float  wr = vr/r*60/(2*PI);  // angular velocity in rpm
    float  wl = vl/r*60/(2*PI);   
 //   Serial.println(wr);
 //   Serial.println(wl);
//    set_speed(wl,wr);
//    set_speedL_PID(wl);
//    set_speedR_PID(wr);
//    int lastCountL = leftEnCount;
//    int lastCountR = rightEnCount;
//    delay(int(T*1000));
    float v1 = 2*PI*get_speedL()/60;
    float v2 = 2*PI*get_speedR()/60;
    x = x + (v1+v2)/2*cos(theta)*T;
    y = y + (v1+v2)/2*sin(theta)*T;
    theta = theta + (v2 - v1)/d*T;
    Serial.println(x);
    Serial.println(y);
    Serial.println(theta);
  }
  stop();
  delay(10000);
}

*/

void goForward(int speed) {
  // Reset encoder counter
 // rightEnCount = 0;
//  leftEnCount = 0;

	// For PWM maximum possible values are 0 to 255
	analogWrite(enR, speed);

//  int motor_L_speed = speed + K*(rightEnCount-leftEnCount);  
//  analogWrite(enL, motor_L_speed);
  analogWrite(enL, speed);
	// Turn on motor A & B
	digitalWrite(inL1, HIGH);
	digitalWrite(inL2, LOW);
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

void leftEnISRA() {
  if (digitalRead(enLB) == HIGH) {
    leftEnCount--;
  } else {
    leftEnCount++;
  }
}

void leftEnISRB() {
  if (digitalRead(enLA) == HIGH) {
    leftEnCount++;
  } else {
    leftEnCount--;
  }
}

void rightEnISRA() {
  if (digitalRead(enRB) == HIGH) {
    rightEnCount--;
  } else {
    rightEnCount++;
  }
}

void rightEnISRB() {
  if (digitalRead(enRA) == HIGH) {
    rightEnCount++;
  } else {
    rightEnCount--;
  }
}
