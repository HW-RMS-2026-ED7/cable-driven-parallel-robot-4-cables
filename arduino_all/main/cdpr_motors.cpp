// ============================================================================
//  cdpr_motors.cpp — Dynamixel motor control, calibration, diagnostics
// ============================================================================

#include "cdpr_motors.h"
#include "cdpr_kinematics.h"
#include "config.h"
#include <Dynamixel2Arduino.h>

using namespace ControlTableItem;

// ─── Shared globals (defined here, declared extern in config.h) ─────────────
float KNOT_OFFSET[4] = {0.003, 0.003, 0.003, 0.003};
float ee_mass_kg     = 0.150;

// ─── Module state ───────────────────────────────────────────────────────────
static Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

static int32_t motor_offset[NUM_MOTORS]     = {0};
static int16_t friction_current[NUM_MOTORS] = {0};
static bool    is_calibrating = false;
static float   last_tensions[4] = {0};

// ─── Tension -> Dynamixel current units ─────────────────────────────────────
static int16_t tensionToCurrent(float tension_N, int motor_idx) {
  float torque    = tension_N * SPOOL_RADIUS;
  float current_A = torque / TORQUE_CONSTANT;
  int16_t dxl_units = (int16_t)(current_A / DXL_CURRENT_UNIT);

  dxl_units += friction_current[motor_idx];
  dxl_units += CURRENT_MARGIN;

  if (dxl_units < 1)           dxl_units = 1;
  if (dxl_units > CURRENT_CAP) dxl_units = CURRENT_CAP;
  return dxl_units;
}

// ─── Init ───────────────────────────────────────────────────────────────────
void cdprMotorsInit() {
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL);

  bool all_ok = true;
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!dxl.ping(DXL_ID[i])) {
      Serial.print(F("ERROR: Motor ")); Serial.print(DXL_ID[i]);
      Serial.println(F(" not responding!"));
      all_ok = false;
      continue;
    }
    dxl.torqueOff(DXL_ID[i]);
    dxl.setOperatingMode(DXL_ID[i], OP_CURRENT_BASED_POSITION);
    dxl.writeControlTableItem(PROFILE_VELOCITY,     DXL_ID[i], PROFILE_VEL);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID[i], PROFILE_ACC);
    dxl.writeControlTableItem(GOAL_CURRENT,         DXL_ID[i], CURRENT_CAP);
    dxl.torqueOn(DXL_ID[i]);
  }

  if (!all_ok)
    Serial.println(F("WARNING: Not all motors responded. Check wiring."));
}

// ─── Move to target ─────────────────────────────────────────────────────────
void moveToTarget(float x, float y, float z, float pitch, float roll) {
  float tensions[4], res_moment[3];
  if (!solveTensions(x, y, z, pitch, roll, tensions, res_moment)) {
    Serial.println(F("ERROR: Degenerate cable configuration. Move rejected."));
    return;
  }

  Serial.print(F("  Tensions (N):  "));
  for (int i = 0; i < 4; i++) { Serial.print(tensions[i], 3); Serial.print(F("  ")); }
  Serial.println();

  Serial.print(F("  Residual M (Nm): Mx="));
  Serial.print(res_moment[0], 5); Serial.print(F("  My="));
  Serial.print(res_moment[1], 5); Serial.print(F("  Mz="));
  Serial.println(res_moment[2], 5);

  float moment_mag = sqrtf(res_moment[0]*res_moment[0]
                         + res_moment[1]*res_moment[1]
                         + res_moment[2]*res_moment[2]);
  if (moment_mag > 0.01) {
    Serial.print(F("  WARNING: |M_residual| = "));
    Serial.print(moment_mag, 4);
    Serial.println(F(" Nm — expect some tilt."));
  }

  float lengths[4], u[4][3], r_vec[4][3];
  computeIK(x, y, z, pitch, roll, lengths, u, r_vec);

  for (int i = 0; i < 4; i++) {
    if (lengths[i] < 0.01f || lengths[i] > 2.0f) {
      Serial.print(F("ERROR: Cable ")); Serial.print(i);
      Serial.print(F(" length = ")); Serial.print(lengths[i], 4);
      Serial.println(F(" m — out of range. Move rejected."));
      return;
    }
  }

  Serial.print(F("  CMD ->  "));
  for (int i = 0; i < 4; i++) {
    int32_t steps   = lengthToSteps(lengths[i]);
    int32_t cmd_pos = MOTOR_DIR[i] * steps + motor_offset[i];
    int16_t cmd_cur = tensionToCurrent(tensions[i], i);

    dxl.writeControlTableItem(GOAL_CURRENT, DXL_ID[i], cmd_cur);
    dxl.setGoalPosition(DXL_ID[i], cmd_pos);
    last_tensions[i] = tensions[i];

    Serial.print(F("M")); Serial.print(DXL_ID[i]);
    Serial.print(F(":pos=")); Serial.print(cmd_pos);
    Serial.print(F(",cur=")); Serial.print(cmd_cur);
    Serial.print(F("  "));
  }
  Serial.println();
}

// ─── Calibration ────────────────────────────────────────────────────────────
bool isCalibratingCDPR() { return is_calibrating; }

void startCalibration() {
  is_calibrating = true;
  Serial.println(F("--- HOME CALIBRATION: constant-tension mode ---"));
  Serial.println(F("Guide the end-effector to the home position, then send 'S'."));

  for (int i = 0; i < NUM_MOTORS; i++) {
    dxl.torqueOff(DXL_ID[i]);
    dxl.setOperatingMode(DXL_ID[i], OP_CURRENT);
    dxl.torqueOn(DXL_ID[i]);
    dxl.setGoalCurrent(DXL_ID[i], MOTOR_DIR[i] * CAL_TENSION_CURRENT);
  }
}

void stopCalibration() {
  if (!is_calibrating) { Serial.println(F("Not calibrating.")); return; }

  Serial.println(F("--- REGISTERING HOME ---"));
  Serial.println(F("Settling..."));
  delay(CAL_SETTLE_MS);

  float home_len[4], home_u[4][3], home_r[4][3];
  computeIK(HOME_X, HOME_Y, HOME_Z, HOME_PITCH, HOME_ROLL,
            home_len, home_u, home_r);

  for (int i = 0; i < NUM_MOTORS; i++) {
    int32_t theo = MOTOR_DIR[i] * lengthToSteps(home_len[i]);
    int32_t raw  = dxl.getPresentPosition(DXL_ID[i]);
    motor_offset[i] = raw - theo;

    dxl.torqueOff(DXL_ID[i]);
    dxl.setOperatingMode(DXL_ID[i], OP_CURRENT_BASED_POSITION);
    dxl.writeControlTableItem(PROFILE_VELOCITY,     DXL_ID[i], PROFILE_VEL);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID[i], PROFILE_ACC);
    dxl.writeControlTableItem(GOAL_CURRENT,         DXL_ID[i], CURRENT_CAP);
    dxl.torqueOn(DXL_ID[i]);
    dxl.setGoalPosition(DXL_ID[i], raw);

    Serial.print(F("  M")); Serial.print(DXL_ID[i]);
    Serial.print(F(" | raw=")); Serial.print(raw);
    Serial.print(F(" | theo=")); Serial.print(theo);
    Serial.print(F(" | offset=")); Serial.println(motor_offset[i]);
  }

  is_calibrating = false;
  Serial.println(F("Home calibration complete."));
}

// ─── Friction identification ────────────────────────────────────────────────
void runFrictionCalibration() {
  Serial.println(F("=== FRICTION IDENTIFICATION ==="));
  Serial.println(F("Keep the end-effector stationary. Motors will twitch briefly."));

  for (int m = 0; m < NUM_MOTORS; m++) {
    Serial.print(F("  Motor ")); Serial.print(DXL_ID[m]); Serial.print(F(" ... "));

    int16_t threshold_pos = 0, threshold_neg = 0;
    int32_t saved_pos = dxl.getPresentPosition(DXL_ID[m]);

    dxl.torqueOff(DXL_ID[m]);
    dxl.setOperatingMode(DXL_ID[m], OP_CURRENT);
    dxl.torqueOn(DXL_ID[m]);

    // Positive ramp
    for (int16_t c = FRIC_CURRENT_STEP; c <= FRIC_CURRENT_MAX; c += FRIC_CURRENT_STEP) {
      dxl.setGoalCurrent(DXL_ID[m], c);
      delay(FRIC_STEP_DELAY_MS);
      if (labs(dxl.getPresentVelocity(DXL_ID[m])) >= FRIC_VEL_THRESHOLD) {
        threshold_pos = c; break;
      }
    }
    dxl.setGoalCurrent(DXL_ID[m], 0); delay(200);

    // Negative ramp
    for (int16_t c = -FRIC_CURRENT_STEP; c >= -FRIC_CURRENT_MAX; c -= FRIC_CURRENT_STEP) {
      dxl.setGoalCurrent(DXL_ID[m], c);
      delay(FRIC_STEP_DELAY_MS);
      if (labs(dxl.getPresentVelocity(DXL_ID[m])) >= FRIC_VEL_THRESHOLD) {
        threshold_neg = labs(c); break;
      }
    }
    dxl.setGoalCurrent(DXL_ID[m], 0); delay(200);

    if (threshold_pos == 0 && threshold_neg == 0) {
      friction_current[m] = 5;
      Serial.println(F("could not detect — using default (5)"));
    } else if (threshold_pos == 0) {
      friction_current[m] = threshold_neg;
    } else if (threshold_neg == 0) {
      friction_current[m] = threshold_pos;
    } else {
      friction_current[m] = (threshold_pos + threshold_neg) / 2;
    }

    Serial.print(F("friction = ")); Serial.print(friction_current[m]);
    Serial.print(F(" units (~")); Serial.print(friction_current[m] * DXL_CURRENT_UNIT * 1000.0f, 1);
    Serial.println(F(" mA)"));

    // Restore Mode 5
    dxl.torqueOff(DXL_ID[m]);
    dxl.setOperatingMode(DXL_ID[m], OP_CURRENT_BASED_POSITION);
    dxl.writeControlTableItem(PROFILE_VELOCITY,     DXL_ID[m], PROFILE_VEL);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID[m], PROFILE_ACC);
    dxl.writeControlTableItem(GOAL_CURRENT,         DXL_ID[m], CURRENT_CAP);
    dxl.torqueOn(DXL_ID[m]);
    dxl.setGoalPosition(DXL_ID[m], saved_pos);
    delay(500);
  }
  Serial.println(F("=== Friction calibration complete ==="));
}

// ─── Weight estimation ──────────────────────────────────────────────────────
void runWeightEstimation() {
  Serial.println(F("=== WEIGHT ESTIMATION ==="));
  Serial.println(F("EE must be at home position and stationary."));
  Serial.println(F("Settling..."));
  delay(CAL_SETTLE_MS);

  float lengths[4], u[4][3], r_vec[4][3];
  computeIK(HOME_X, HOME_Y, HOME_Z, HOME_PITCH, HOME_ROLL,
            lengths, u, r_vec);

  float total_Fz = 0;
  for (int i = 0; i < NUM_MOTORS; i++) {
    int16_t raw_current = dxl.getPresentCurrent(DXL_ID[i]);
    float current_A = fabsf((float)raw_current * DXL_CURRENT_UNIT);
    float friction_A = (float)friction_current[i] * DXL_CURRENT_UNIT;
    float useful_A = max(0.0f, current_A - friction_A);

    float tension = (useful_A * TORQUE_CONSTANT) / SPOOL_RADIUS;
    float Fz_i = tension * u[i][2];
    total_Fz += Fz_i;

    Serial.print(F("  M")); Serial.print(DXL_ID[i]);
    Serial.print(F(": I=")); Serial.print(current_A * 1000, 1);
    Serial.print(F("mA  T=")); Serial.print(tension, 3);
    Serial.print(F("N  Fz=")); Serial.print(Fz_i, 3);
    Serial.println(F("N"));
  }

  float est_mass = total_Fz / GRAVITY;
  Serial.print(F("  Total Fz = ")); Serial.print(total_Fz, 3); Serial.println(F(" N"));
  Serial.print(F("  Estimated mass = ")); Serial.print(est_mass * 1000, 1); Serial.println(F(" g"));

  if (est_mass > 0.01) {
    float old_mass = ee_mass_kg;
    ee_mass_kg = est_mass;
    Serial.print(F("  Mass updated: ")); Serial.print(old_mass * 1000, 1);
    Serial.print(F(" g -> ")); Serial.print(ee_mass_kg * 1000, 1); Serial.println(F(" g"));
  } else {
    Serial.println(F("  Estimated mass too low — keeping previous value."));
  }
  Serial.println(F("=== Weight estimation complete ==="));
}

// ─── Parameter setters ──────────────────────────────────────────────────────
void setKnotOffset(int idx, float mm) {
  if (idx >= 0 && idx < 4) {
    KNOT_OFFSET[idx] = mm / 1000.0f;
    Serial.print(F("Knot offset[")); Serial.print(idx);
    Serial.print(F("] = ")); Serial.print(mm, 2); Serial.println(F(" mm"));
  } else {
    Serial.println(F("Invalid index (0-3)."));
  }
}

void setMass(float grams) {
  if (grams > 0) {
    ee_mass_kg = grams / 1000.0f;
    Serial.print(F("Mass set to ")); Serial.print(grams, 1); Serial.println(F(" g"));
  } else {
    Serial.println(F("Invalid mass."));
  }
}

// ─── Diagnostics ────────────────────────────────────────────────────────────
void printDiagnostics() {
  Serial.println(F("--- DIAGNOSTICS ---"));
  for (int i = 0; i < NUM_MOTORS; i++) {
    int32_t pos = dxl.getPresentPosition(DXL_ID[i]);
    int16_t cur = dxl.getPresentCurrent(DXL_ID[i]);
    int32_t vel = dxl.getPresentVelocity(DXL_ID[i]);

    Serial.print(F("  M")); Serial.print(DXL_ID[i]);
    Serial.print(F(": pos=")); Serial.print(pos);
    Serial.print(F("  cur=")); Serial.print(cur);
    Serial.print(F(" (")); Serial.print(cur * DXL_CURRENT_UNIT * 1000, 1); Serial.print(F("mA)"));
    Serial.print(F("  vel=")); Serial.print(vel);
    Serial.print(F("  fric=")); Serial.print(friction_current[i]);
    Serial.print(F("  offset=")); Serial.println(motor_offset[i]);
  }
  Serial.print(F("  ee_mass = ")); Serial.print(ee_mass_kg * 1000, 1); Serial.println(F(" g"));
  Serial.print(F("  Last T (N): "));
  for (int i = 0; i < 4; i++) { Serial.print(last_tensions[i], 3); Serial.print(F("  ")); }
  Serial.println();
  Serial.println(F("-------------------"));
}

void printConfig() {
  Serial.println(F("--- Configuration ---"));
  Serial.print(F("  Frame:        ")); Serial.print(FRAME_WIDTH*100,1);
  Serial.print(F(" x ")); Serial.print(FRAME_HEIGHT*100,1); Serial.println(F(" cm"));
  Serial.print(F("  End-effector: ")); Serial.print(EE_WIDTH*100,1);
  Serial.print(F(" x ")); Serial.print(EE_LENGTH*100,1); Serial.println(F(" cm"));
  Serial.print(F("  Spool r:      ")); Serial.print(SPOOL_RADIUS*1000,1); Serial.println(F(" mm"));
  Serial.print(F("  EE mass:      ")); Serial.print(ee_mass_kg*1000,0); Serial.println(F(" g"));
  Serial.print(F("  Kt_eff:       ")); Serial.print(TORQUE_CONSTANT,3); Serial.println(F(" Nm/A"));
  Serial.print(F("  Knot offsets:  "));
  for (int i = 0; i < 4; i++) { Serial.print(KNOT_OFFSET[i]*1000,1); Serial.print(F("mm  ")); }
  Serial.println();
  Serial.print(F("  T_min:        ")); Serial.print(T_MIN_N,2); Serial.println(F(" N"));
  Serial.print(F("  Home: ")); Serial.print(HOME_X,3); Serial.print(F(", "));
  Serial.print(HOME_Y,3); Serial.print(F(", ")); Serial.print(HOME_Z,3);
  Serial.print(F(", p=")); Serial.print(HOME_PITCH,3);
  Serial.print(F(", r=")); Serial.println(HOME_ROLL,3);
  Serial.println(F("---------------------"));
}

void printHelp() {
  Serial.println(F("=== CDPR + Gripper Controller ==="));
  Serial.println(F("CDPR commands:"));
  Serial.println(F("  C           Start home calibration (constant tension)"));
  Serial.println(F("  S           Stop calibration & register home"));
  Serial.println(F("  F           Run friction identification"));
  Serial.println(F("  W           Run weight estimation"));
  Serial.println(F("  D           Print diagnostics"));
  Serial.println(F("  I           Print configuration"));
  Serial.println(F("  K i val     Set knot offset (mm): K 0 3.5"));
  Serial.println(F("  M val       Set mass (g):         M 250"));
  Serial.println(F("  X,Y,Z,P,R  Move to pose: 0.0,0.0,-0.2,0.0,0.0"));
  Serial.println(F("  H           Move to home position"));
  Serial.println(F("Gripper commands:"));
  Serial.println(F("  GO          Open gripper"));
  Serial.println(F("  GC          Close gripper"));
  Serial.println(F("  GJ+ / GJ-  Jog gripper open / close (calibration)"));
  Serial.println(F("  GSO         Save current pos as OPEN"));
  Serial.println(F("  GSC         Save current pos as CLOSE"));
  Serial.println(F("  GP          Print gripper positions"));
  Serial.println(F("  ?           Print this help"));
  Serial.println();
}
