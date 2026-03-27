// ============================================================================
//  gripper.cpp — Non-blocking stepper gripper with trapezoidal acceleration
// ============================================================================
//
//  The original gripper code used a blocking for-loop with delayMicroseconds().
//  This version uses a state machine: each call to gripperUpdate() produces
//  at most one step pulse, then returns immediately so the main loop can
//  also service the Dynamixel motors and serial commands without stalling.
//
// ============================================================================

#include "gripper.h"
#include "config.h"
#include <EEPROM.h>

// ─── State ──────────────────────────────────────────────────────────────────

static long currentPos = 0;
static long openPos    = 0;
static long closePos   = 0;
static bool limitsSet  = false;

// Motion state machine
static bool     moving       = false;
static long     stepsRemaining = 0;
static long     totalSteps   = 0;
static long     stepIndex    = 0;
static int8_t   stepDir      = 0;      // +1 or -1
static uint32_t lastStepUs   = 0;      // micros() of last pulse

// ─── Init ───────────────────────────────────────────────────────────────────

void gripperInit() {
  pinMode(GRIPPER_STEP_PIN, OUTPUT);
  pinMode(GRIPPER_DIR_PIN,  OUTPUT);

  EEPROM.get(EEPROM_OPEN_ADDR,  openPos);
  EEPROM.get(EEPROM_CLOSE_ADDR, closePos);
  limitsSet = (openPos != closePos);

  if (limitsSet) {
    Serial.print(F("[Gripper] Loaded OPEN=")); Serial.print(openPos);
    Serial.print(F("  CLOSE=")); Serial.println(closePos);
  } else {
    Serial.println(F("[Gripper] No saved positions in EEPROM"));
  }
}

// ─── Internal: start a move of N steps in a direction ───────────────────────

static void beginMove(long steps, int8_t dir) {
  if (steps <= 0) return;
  moving         = true;
  totalSteps     = steps;
  stepsRemaining = steps;
  stepIndex      = 0;
  stepDir        = dir;
  lastStepUs     = micros();

  digitalWrite(GRIPPER_DIR_PIN, dir > 0 ? HIGH : LOW);
}

// ─── Update (call every loop) ───────────────────────────────────────────────

void gripperUpdate() {
  if (!moving) return;

  // Compute current delay from trapezoidal profile
  long stepsFromEnd = stepsRemaining - 1;
  long rampPhase    = min(stepIndex, stepsFromEnd);
  int d;
  if (rampPhase < GRIPPER_ACCEL_STEPS) {
    d = GRIPPER_START_DELAY - (long)(GRIPPER_START_DELAY - GRIPPER_MIN_DELAY) * rampPhase / GRIPPER_ACCEL_STEPS;
  } else {
    d = GRIPPER_MIN_DELAY;
  }

  // Wait for the inter-step interval (non-blocking check)
  uint32_t now = micros();
  if ((now - lastStepUs) < (uint32_t)(d * 2)) return;  // *2 because HIGH+LOW phase

  // Pulse
  digitalWrite(GRIPPER_STEP_PIN, HIGH);
  delayMicroseconds(d);
  digitalWrite(GRIPPER_STEP_PIN, LOW);
  // LOW phase will be covered by the next iteration's wait

  lastStepUs = micros();
  currentPos += stepDir;
  stepIndex++;
  stepsRemaining--;

  if (stepsRemaining <= 0) {
    moving = false;
  }
}

bool gripperBusy() { return moving; }

// ─── Motion commands ────────────────────────────────────────────────────────

void gripperGoOpen() {
  if (!limitsSet) { Serial.println(F("[Gripper] Calibrate first!")); return; }
  if (moving)     { Serial.println(F("[Gripper] Already moving")); return; }
  long delta = openPos - currentPos;
  if (delta == 0) { Serial.println(F("[Gripper] Already at OPEN")); return; }
  Serial.print(F("[Gripper] -> OPEN (")); Serial.print(delta); Serial.println(F(" steps)"));
  beginMove(labs(delta), delta > 0 ? 1 : -1);
}

void gripperGoClose() {
  if (!limitsSet) { Serial.println(F("[Gripper] Calibrate first!")); return; }
  if (moving)     { Serial.println(F("[Gripper] Already moving")); return; }
  long delta = closePos - currentPos;
  if (delta == 0) { Serial.println(F("[Gripper] Already at CLOSE")); return; }
  Serial.print(F("[Gripper] -> CLOSE (")); Serial.print(delta); Serial.println(F(" steps)"));
  beginMove(labs(delta), delta > 0 ? 1 : -1);
}

void gripperJog(bool openDir) {
  if (moving) { Serial.println(F("[Gripper] Already moving")); return; }
  Serial.print(F("[Gripper] Jog ")); Serial.println(openDir ? F("open") : F("close"));
  beginMove(GRIPPER_STEPS_PER_JOG, openDir ? 1 : -1);
}

// ─── Save / print ───────────────────────────────────────────────────────────

void gripperSaveOpen() {
  openPos = currentPos;
  EEPROM.put(EEPROM_OPEN_ADDR, openPos);
  limitsSet = (openPos != closePos);
  Serial.print(F("[Gripper] OPEN saved: ")); Serial.println(openPos);
}

void gripperSaveClose() {
  closePos = currentPos;
  EEPROM.put(EEPROM_CLOSE_ADDR, closePos);
  limitsSet = (openPos != closePos);
  Serial.print(F("[Gripper] CLOSE saved: ")); Serial.println(closePos);
}

void gripperPrintPositions() {
  Serial.print(F("[Gripper] Current: ")); Serial.println(currentPos);
  Serial.print(F("[Gripper] Open:    ")); Serial.println(openPos);
  Serial.print(F("[Gripper] Close:   ")); Serial.println(closePos);
  Serial.print(F("[Gripper] Moving:  ")); Serial.println(moving ? F("yes") : F("no"));
}
