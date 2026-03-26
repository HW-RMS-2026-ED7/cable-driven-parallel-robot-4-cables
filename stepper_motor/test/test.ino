#include <EEPROM.h>

const int stepPin = 7;
const int dirPin  = 4;

// --- EEPROM addresses ---
const int EEPROM_OPEN_ADDR  = 0;
const int EEPROM_CLOSE_ADDR = 4;

// --- Position tracking ---
long currentPos    = 0;    // tracks absolute position in steps
long openPos       = 0;    // will be set during calibration
long closePos      = 0;    // will be set during calibration
bool limitsSet     = false;

// --- Motion parameters ---
const int stepsPerJog  = 200;   // small jog for calibration
const int stepsDelay   = 500;   // cruise speed (µs between pulses)

// --- Acceleration ---
const int startDelay   = 2000;  // slow start (µs) — adjust to taste
const int minDelay     = 300;   // max speed (µs)
const int accelSteps   = 80;    // how many steps to ramp over (short = aggressive)

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  Serial.begin(115200);
  
  // Load positions from EEPROM
  EEPROM.get(EEPROM_OPEN_ADDR, openPos);
  EEPROM.get(EEPROM_CLOSE_ADDR, closePos);
  limitsSet = (openPos != closePos);
  
  Serial.println(F("=== Gripper Calibration ==="));
  Serial.println(F("  o/l : jog open/close direction"));
  Serial.println(F("  s   : save current pos as OPEN"));
  Serial.println(F("  c   : save current pos as CLOSE"));
  Serial.println(F("  1   : go to OPEN position"));
  Serial.println(F("  2   : go to CLOSE position"));
  Serial.println(F("  p   : print positions"));
  
  // Show loaded values
  if (limitsSet) {
    Serial.print(F("Loaded OPEN: ")); Serial.println(openPos);
    Serial.print(F("Loaded CLOSE: ")); Serial.println(closePos);
  } else {
    Serial.println(F("No saved positions in EEPROM"));
  }
}

// Move a given number of steps with trapezoidal acceleration
void moveSteps(long steps, bool dir) {
  digitalWrite(dirPin, dir ? HIGH : LOW);
  long absSteps = abs(steps);

  for (long i = 0; i < absSteps; i++) {
    // --- compute delay with accel/decel ramp ---
    int d;
    long stepsFromEnd = absSteps - 1 - i;
    long rampPhase    = min(i, stepsFromEnd);  // shortest distance to either end

    if (rampPhase < accelSteps) {
      // linear ramp: startDelay → minDelay
      d = startDelay - (long)(startDelay - minDelay) * rampPhase / accelSteps;
    } else {
      d = minDelay;  // cruise
    }

    digitalWrite(stepPin, HIGH);
    delayMicroseconds(d);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(d);

    // update absolute position
    currentPos += dir ? 1 : -1;
  }
}

void loop() {
  if (Serial.available() <= 0) return;
  char ch = Serial.read();

  // --- Jog for calibration ---
  if (ch == 'o' || ch == 'O') {
    Serial.print(F("Jog open... "));
    moveSteps(stepsPerJog, HIGH);
    Serial.println(currentPos);
  }
  else if (ch == 'l' || ch == 'L') {
    Serial.print(F("Jog close... "));
    moveSteps(stepsPerJog, LOW);
    Serial.println(currentPos);
  }

  // --- Save limits ---
  else if (ch == 's' || ch == 'S') {
    openPos = currentPos;
    EEPROM.put(EEPROM_OPEN_ADDR, openPos);
    Serial.print(F("OPEN position saved: ")); Serial.println(openPos);
    limitsSet = (openPos != closePos);
  }
  else if (ch == 'c' || ch == 'C') {
    closePos = currentPos;
    EEPROM.put(EEPROM_CLOSE_ADDR, closePos);
    Serial.print(F("CLOSE position saved: ")); Serial.println(closePos);
    limitsSet = (openPos != closePos);
  }

  // --- Go to saved positions ---
  else if (ch == '1') {
    if (!limitsSet) { Serial.println(F("Calibrate first!")); return; }
    long delta = openPos - currentPos;
    Serial.print(F("-> OPEN (")); Serial.print(delta); Serial.println(F(" steps)"));
    moveSteps(abs(delta), delta > 0);
  }
  else if (ch == '2') {
    if (!limitsSet) { Serial.println(F("Calibrate first!")); return; }
    long delta = closePos - currentPos;
    Serial.print(F("-> CLOSE (")); Serial.print(delta); Serial.println(F(" steps)"));
    moveSteps(abs(delta), delta > 0);
  }

  // --- Debug ---
  else if (ch == 'p' || ch == 'P') {
    Serial.print(F("Current: ")); Serial.println(currentPos);
    Serial.print(F("Open:    ")); Serial.println(openPos);
    Serial.print(F("Close:   ")); Serial.println(closePos);
  }
}