// ============================================================================
//  config.h — All tunable parameters in one place
// ============================================================================
#pragma once

#include <Arduino.h>

// --- Frame geometry (metres) ------------------------------------------------
const float FRAME_WIDTH  = 0.40;
const float FRAME_HEIGHT = 0.40;

// --- End-effector (EE) geometry (metres) ------------------------------------
const float EE_WIDTH  = 0.08;
const float EE_LENGTH = 0.08;

// --- Per-cable knot / attachment offsets (metres) ---------------------------
//  Positive = knot eats cable (motor must pay out more).
//  ORDER: cable 0..3, matching DXL_ID[] order.
extern float KNOT_OFFSET[4];

// --- Spool ------------------------------------------------------------------
const float SPOOL_RADIUS = 0.03;  // metres

// --- Mass & gravity ---------------------------------------------------------
extern float ee_mass_kg;          // updated by weight-estimation
const float GRAVITY = 9.81;

// --- Dynamixel XM430-W210-T ------------------------------------------------
const float DXL_STEPS_PER_REV = 4096.0;
const float TORQUE_CONSTANT    = 1.29;      // Nm/A
const float DXL_CURRENT_UNIT   = 0.00269;   // A per Dxl unit

// --- Pre-tension ------------------------------------------------------------
const float T_MIN_N = 0.20;  // N

// --- Current margin & cap ---------------------------------------------------
const int16_t CURRENT_MARGIN = 15;
const int16_t CURRENT_CAP    = 800;

// --- Motion profile (Dynamixel units) ---------------------------------------
const uint32_t PROFILE_VEL = 200;
const uint32_t PROFILE_ACC = 50;

// --- Calibration constants --------------------------------------------------
const int16_t  CAL_TENSION_CURRENT = -25;
const uint32_t CAL_SETTLE_MS       = 2000;

// --- Home position ----------------------------------------------------------
const float HOME_X     =  0.0;
const float HOME_Y     =  0.0;
const float HOME_Z     = -0.46;
const float HOME_PITCH =  0.0;
const float HOME_ROLL  =  0.0;

// --- Friction calibration ---------------------------------------------------
const int16_t  FRIC_CURRENT_STEP  = 2;
const int16_t  FRIC_CURRENT_MAX   = 80;
const uint32_t FRIC_STEP_DELAY_MS = 300;
const int32_t  FRIC_VEL_THRESHOLD = 3;

// --- Hardware wiring --------------------------------------------------------
#define DXL_SERIAL Serial3
const int DXL_DIR_PIN = 84;

const uint8_t NUM_MOTORS = 4;
const uint8_t DXL_ID[NUM_MOTORS]   = {1, 3, 4, 2};
const int8_t  MOTOR_DIR[NUM_MOTORS] = {1, 1, 1, 1};

const float DXL_PROTOCOL = 2.0;

// --- Anchor layout (frame plane Z = 0) -------------------------------------
//     A0 (+X,+Y)   A1 (-X,+Y)
//     A3 (+X,-Y)   A2 (-X,-Y)
const float ANCHOR[4][3] = {
  { FRAME_WIDTH / 2,  FRAME_HEIGHT / 2, 0},
  {-FRAME_WIDTH / 2,  FRAME_HEIGHT / 2, 0},
  {-FRAME_WIDTH / 2, -FRAME_HEIGHT / 2, 0},
  { FRAME_WIDTH / 2, -FRAME_HEIGHT / 2, 0}
};

// --- EE attachment in body frame (same corner order) ------------------------
const float EE_LOCAL[4][3] = {
  { EE_WIDTH / 2,  EE_LENGTH / 2, 0},
  {-EE_WIDTH / 2,  EE_LENGTH / 2, 0},
  {-EE_WIDTH / 2, -EE_LENGTH / 2, 0},
  { EE_WIDTH / 2, -EE_LENGTH / 2, 0}
};

// --- Gripper (stepper) ------------------------------------------------------
const int GRIPPER_STEP_PIN = 7;
const int GRIPPER_DIR_PIN  = 4;

const int  GRIPPER_STEPS_PER_JOG = 1000;
const int  GRIPPER_START_DELAY   = 2000;  // us — slow start
const int  GRIPPER_MIN_DELAY     = 300;   // us — max speed
const int  GRIPPER_ACCEL_STEPS   = 80;    // ramp length

// EEPROM addresses for gripper positions
const int EEPROM_OPEN_ADDR  = 0;
const int EEPROM_CLOSE_ADDR = 4;
