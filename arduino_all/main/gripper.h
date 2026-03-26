// ============================================================================
//  gripper.h — Non-blocking stepper gripper with EEPROM calibration
// ============================================================================
#pragma once

#include <Arduino.h>

// Call once in setup()
void gripperInit();

// Call every loop() iteration — drives the stepper state machine
void gripperUpdate();

// True while gripper is moving
bool gripperBusy();

// Motion commands (non-blocking — returns immediately, gripperUpdate() does the work)
void gripperGoOpen();
void gripperGoClose();
void gripperJog(bool openDir);

// Save current position as the OPEN or CLOSE limit
void gripperSaveOpen();
void gripperSaveClose();

// Print current / saved positions
void gripperPrintPositions();
