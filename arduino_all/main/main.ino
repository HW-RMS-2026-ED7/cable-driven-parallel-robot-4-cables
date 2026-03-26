// ============================================================================
//  CDPR + Gripper Unified Controller
//  Hardware: OpenCR 1.0  +  4x Dynamixel XM430-W210-T  +  Stepper gripper
// ============================================================================
//
//  All subsystems run non-blocking from a single loop().
//  The gripper uses a state machine (gripperUpdate) instead of blocking delays.
//  Dynamixel commands are inherently non-blocking (fire-and-forget via Mode 5).
//
// ============================================================================

#include "config.h"
#include "cdpr_motors.h"
#include "cdpr_kinematics.h"
#include "gripper.h"

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  cdprMotorsInit();
  gripperInit();

  printHelp();
  printConfig();
}

void loop() {
  // 1. Service the gripper stepper (one pulse per call, non-blocking)
  gripperUpdate();

  // 2. Check for serial commands
  if (Serial.available() <= 0) return;

  char first = Serial.peek();

  // --- Gripper commands (all start with 'G') --------------------------------
  if (first == 'G' || first == 'g') {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "GO")       gripperGoOpen();
    else if (cmd == "GC")  gripperGoClose();
    else if (cmd == "GJ+") gripperJog(true);
    else if (cmd == "GJ-") gripperJog(false);
    else if (cmd == "GSO") gripperSaveOpen();
    else if (cmd == "GSC") gripperSaveClose();
    else if (cmd == "GP")  gripperPrintPositions();
    else {
      Serial.println(F("Unknown gripper command. Use GO/GC/GJ+/GJ-/GSO/GSC/GP"));
    }
    return;
  }

  // --- CDPR single-character commands ---------------------------------------
  if (first == 'C' || first == 'c') {
    Serial.readStringUntil('\n');
    startCalibration();
    return;
  }
  if (first == 'S' || first == 's') {
    Serial.readStringUntil('\n');
    stopCalibration();
    return;
  }
  if (first == 'F' || first == 'f') {
    Serial.readStringUntil('\n');
    if (isCalibratingCDPR()) { Serial.println(F("Finish home cal first.")); return; }
    runFrictionCalibration();
    return;
  }
  if (first == 'W' || first == 'w') {
    Serial.readStringUntil('\n');
    if (isCalibratingCDPR()) { Serial.println(F("Finish home cal first.")); return; }
    runWeightEstimation();
    return;
  }
  if (first == 'D' || first == 'd') {
    Serial.readStringUntil('\n');
    printDiagnostics();
    return;
  }
  if (first == 'I' || first == 'i') {
    Serial.readStringUntil('\n');
    printConfig();
    return;
  }
  if (first == '?') {
    Serial.readStringUntil('\n');
    printHelp();
    return;
  }
  if (first == 'H' || first == 'h') {
    Serial.readStringUntil('\n');
    if (isCalibratingCDPR()) { Serial.println(F("Finish home cal first.")); return; }
    Serial.println(F("Moving to home..."));
    moveToTarget(HOME_X, HOME_Y, HOME_Z, HOME_PITCH, HOME_ROLL);
    return;
  }

  // --- Knot offset:  K <index> <mm> ----------------------------------------
  if (first == 'K' || first == 'k') {
    Serial.read();
    int idx = Serial.parseInt();
    float mm = Serial.parseFloat();
    Serial.readStringUntil('\n');
    setKnotOffset(idx, mm);
    return;
  }

  // --- Manual mass:  M <grams> ---------------------------------------------
  if (first == 'M' || first == 'm') {
    Serial.read();
    float g = Serial.parseFloat();
    Serial.readStringUntil('\n');
    setMass(g);
    return;
  }

  // --- Pose:  X,Y,Z,P,R ---------------------------------------------------
  if (isCalibratingCDPR()) {
    Serial.readStringUntil('\n');
    Serial.println(F("Calibrating — send 'S' to finish first."));
    return;
  }

  if ((first >= '0' && first <= '9') || first == '-' || first == '+' || first == '.') {
    float tx = Serial.parseFloat();
    float ty = Serial.parseFloat();
    float tz = Serial.parseFloat();
    float tp = Serial.parseFloat();
    float tr = Serial.parseFloat();
    Serial.readStringUntil('\n');

    Serial.print(F("Target: ")); Serial.print(tx,4); Serial.print(F(", "));
    Serial.print(ty,4); Serial.print(F(", ")); Serial.print(tz,4);
    Serial.print(F(", p=")); Serial.print(tp,4);
    Serial.print(F(", r=")); Serial.println(tr,4);

    moveToTarget(tx, ty, tz, tp, tr);
    return;
  }

  // Unknown
  Serial.readStringUntil('\n');
  Serial.println(F("Unknown command. Send '?' for help."));
}
