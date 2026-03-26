// Define stepper motor control pins based on your setup
const int stepPin = 7;
const int dirPin = 4;

// Define how many steps to move per key press (adjust this to change what "a bit" means)
const int stepsPerMove = 1000; 
const int stepsDelay = 5000;

void setup() {
  // Configure the control pins as outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  
  // Start serial communication
  Serial.begin(115200);
  Serial.println("Teleoperation Initialized.");
  Serial.println("Send 'f' to move forward, 'b' to move backward.");
}

void loop() {
  // Check if data is available to read from the Serial Monitor
  if (Serial.available() > 0) {
    // Read the incoming byte
    char incomingChar = Serial.read();

    // --------------------------------------------------
    // 1. Move Forward (Clockwise)
    // --------------------------------------------------
    if (incomingChar == 'o' || incomingChar == 'O') {
      Serial.println("Moving Forward...");
      digitalWrite(dirPin, HIGH); // Set direction

      for (int x = 0; x < stepsPerMove; x++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(stepsDelay);    
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepsDelay);    
      }
    }
    
    // --------------------------------------------------
    // 2. Move Backward (Counter-Clockwise)
    // --------------------------------------------------
    else if (incomingChar == 'l' || incomingChar == 'L') {
      Serial.println("Moving Backward...");
      digitalWrite(dirPin, LOW);  // Reverse direction

      for (int x = 0; x < stepsPerMove; x++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(stepsDelay);     
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepsDelay);
      }
    }
  }
}