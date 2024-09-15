// Left wheel
int in1PinL = 8;  
int in2PinL = 9; 
int enablePinL = 7;  

// Right wheel
int in1PinR = 11;  // INOUT 3
int in2PinR = 10;  // INOUT 4
int enablePinR = 6;  

// For encoder
int enLA = 18;
int enLB = 19;

int enRA = 2;
int enRB = 3;


void setup() {
  // Set the motor control pins as outputs
  pinMode(in1PinL, OUTPUT);
  pinMode(in2PinL, OUTPUT);
  pinMode(enablePinL, OUTPUT);
  
  pinMode(in1PinR, OUTPUT);
  pinMode(in2PinR, OUTPUT);
  pinMode(enablePinR, OUTPUT);
    
}

void go_straight() {
  // Set the motor speed (PWM value)
  analogWrite(enablePinL, 255); // speed: 0 -> 255
  analogWrite(enablePinR, 255); 
  
  // Start both motors going straight
  digitalWrite(in1PinL, HIGH);  // Motor A forward
  digitalWrite(in2PinL, LOW);   
  
  digitalWrite(in1PinR, HIGH);  // Motor B forward
  digitalWrite(in2PinR, LOW); 
}


void go_backward() {
  // Set the motor speed (PWM value)
  analogWrite(enablePinL, 128); // Full speed for motor A
  analogWrite(enablePinR, 255); // Full speed for motor B
  
  // Reverse both motors (backward)
  digitalWrite(in1PinL, LOW);   // Motor A backward
  digitalWrite(in2PinL, HIGH);  
  
  digitalWrite(in1PinR, LOW);   // Motor B backward
  digitalWrite(in2PinR, HIGH);  
}


void spinning() {
  analogWrite(enablePinL, 255); // Full speed for motor A
  analogWrite(enablePinR, 255); // Full speed for motor B
  
  // Reverse both motors (backward)
  digitalWrite(in1PinL, LOW);   // Motor A backward
  digitalWrite(in2PinL, HIGH);  

  digitalWrite(in1PinR, HIGH);  // Motor B forward
  digitalWrite(in2PinR, LOW); 
}

void stop() {
  digitalWrite(in1PinL, LOW);  
  digitalWrite(in2PinL, LOW);   
  
  digitalWrite(in1PinR, LOW);  
  digitalWrite(in2PinR, LOW); 
}

void loop() {

  // remember to turn on the battery before running the code, otherwise, it wont work

  // stop();         // Stop
  // delay(5000);    // Wait for 2 seconds


  go_straight();  // Go straight forward

  // spinning();
  // delay(10000);    // Move forward for 5 seconds
  
  // stop();         // Stop
  // delay(2000);    // Wait for 2 seconds
  
  // go_backward();  // Go backward
  // delay(5000);    // Move backward for 5 seconds
  
  // stop();         // Stop
  // delay(2000);    // Wait for 2 seconds

}

// note: motor A is weaker/ has less traction?
