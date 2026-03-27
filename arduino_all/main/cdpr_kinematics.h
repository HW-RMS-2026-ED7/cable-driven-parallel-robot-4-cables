// ============================================================================
//  cdpr_kinematics.h — Inverse kinematics & 6-DOF wrench solver
// ============================================================================
#pragma once

#include <Arduino.h>

// Inverse kinematics: pose -> cable lengths, unit vectors, moment arms
void computeIK(float x, float y, float z, float pitch, float roll,
               float lengths[4], float u[4][3], float r[4][3]);

// Cable length (metres) -> Dynamixel encoder steps
int32_t lengthToSteps(float length_m);

// Least-squares wrench solver with positive-tension redistribution.
// Returns true on success.  tensions_out[4] and residual_moment[3] filled.
bool solveTensions(float x, float y, float z, float pitch, float roll,
                   float tensions_out[4], float residual_moment[3]);
