// ============================================================================
//  cdpr_motors.h — Dynamixel motor control, calibration, diagnostics
// ============================================================================
#pragma once

#include <Arduino.h>

// Call once in setup()
void cdprMotorsInit();

// Move EE to target pose (x, y, z, pitch, roll)
void moveToTarget(float x, float y, float z, float pitch, float roll);

// Home calibration (two-step: start then stop)
void startCalibration();
void stopCalibration();
bool isCalibratingCDPR();

// Friction identification (blocking — motors twitch briefly)
void runFrictionCalibration();

// Weight estimation (blocking — reads motor currents)
void runWeightEstimation();

// Set knot offset for cable idx (in mm)
void setKnotOffset(int idx, float mm);

// Set EE mass (in grams)
void setMass(float grams);

// Print diagnostics / config / help
void printDiagnostics();
void printConfig();
void printHelp();
