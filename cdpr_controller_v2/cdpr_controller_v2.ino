// ============================================================================
//  Cable-Driven Parallel Robot (CDPR) Controller v2.0
//  Hardware: OpenCR 1.0  +  4× Dynamixel XM430-W210-T
// ============================================================================
//
//  WHAT'S NEW vs v1:
//    1. Full 6-DOF wrench solver (force + moment balance) — minimises tilt
//    2. Per-cable tension → current feedforward via Mode 5
//    3. Per-cable knot / attachment offsets (easily editable)
//    4. Friction identification routine per motor
//    5. Weight estimation routine (adapts to gripper payload)
//    6. Tension redistribution to guarantee all-positive tensions
//    7. Residual moment reporting (shows expected tilt)
//    8. Catenary-free (fishing line is inextensible at these loads)
//
//  CONTROL PHILOSOPHY:
//    Mode 5 (Current-Based Position Control) commands BOTH a goal position
//    AND a goal current.  We set:
//      • Goal Position  = geometric cable length from IK  (+ calibration offset)
//      • Goal Current   = required cable tension converted to motor current
//                         + friction compensation + safety margin
//    This gives hybrid position-tension control: the motor drives toward the
//    correct length but never exceeds the force budget for that cable.
//    The wrench solver distributes tensions to minimise residual moments,
//    which is the main lever against tilt in a 4-cable / 6-DOF system.
//
// ============================================================================

#include <Dynamixel2Arduino.h>
#include <math.h>

#if USE_IMU
#include <IMU.h>
cIMU imu;
#endif

// ████████████████████████████████████████████████████████████████████████████
//  SECTION 1 — EASY-TO-CHANGE PARAMETERS
//  Modify these freely.  Units in comments.
// ████████████████████████████████████████████████████████████████████████████

// --- Frame geometry (metres) -----------------------------------------------
//  Measured between the cable exit points (pulley centres / spool axes).
const float FRAME_WIDTH  = 0.40;   // X span  (m)
const float FRAME_HEIGHT = 0.40;   // Y span  (m)

// --- End-effector (EE) geometry (metres) -----------------------------------
//  Distance between the attachment holes on the platform.
const float EE_WIDTH  = 0.08;     // X span  (m)
const float EE_LENGTH = 0.08;     // Y span  (m)

// --- Per-cable knot / attachment offsets (metres) --------------------------
//  Extra cable length consumed by each knot + any rigging asymmetry.
//  Positive = knot eats cable (motor must pay out more to reach same point).
//  Measure with a ruler: tie your knots, measure from the spool exit to the
//  EE attachment hole, subtract the straight-line distance.
//  ORDER: cable 0 … 3, matching DXL_ID[] order below.
float KNOT_OFFSET[4] = {
  0.003,   // Cable 0  (m)  — adjust after measuring
  0.003,   // Cable 1
  0.003,   // Cable 2
  0.003    // Cable 3
};

// --- Spool -----------------------------------------------------------------
const float SPOOL_RADIUS = 0.03;  // metres

// --- Mass & gravity --------------------------------------------------------
float ee_mass_kg = 0.150;         // Will be updated by weight-estimation
const float GRAVITY = 9.81;

// --- Dynamixel XM430-W210-T -----------------------------------------------
const float DXL_STEPS_PER_REV = 4096.0;
//  Effective torque constant  Kt_eff = stall_torque / stall_current
//  XM430-W210-T @ 12 V:  3.0 Nm / 2.3 A ≈ 1.30 Nm/A
//  This maps input current to output-shaft torque (includes gearbox).
const float TORQUE_CONSTANT = 1.30;   // Nm / A
//  Current resolution: 1 Dynamixel unit = 2.69 mA
const float DXL_CURRENT_UNIT = 0.00269;  // A per unit

// --- Pre-tension -----------------------------------------------------------
//  Minimum cable tension (N) we always maintain.  Prevents slack.
const float T_MIN_N = 0.20;

// --- Current margin --------------------------------------------------------
//  Extra current (Dynamixel units) added on top of computed tension current
//  so the motor can actually reach the goal position.
const int16_t CURRENT_MARGIN = 15;     // ~40 mA headroom

// --- Absolute current cap (Dynamixel units) --------------------------------
//  Safety ceiling.  XM430 max is ~1193 mA ≈ 443 units.
const int16_t CURRENT_CAP = 200;       // ~538 mA

// --- Motion profile (Dynamixel units) --------------------------------------
const uint32_t PROFILE_VEL  = 200;
const uint32_t PROFILE_ACC  = 50;

// --- Calibration constants -------------------------------------------------
const int16_t  CAL_TENSION_CURRENT = -30;   // Constant-current reel-in during home cal
const uint32_t CAL_SETTLE_MS       = 2000;  // Wait time before reading positions

// --- Home position ---------------------------------------------------------
const float HOME_X     =  0.0;
const float HOME_Y     =  0.0;
const float HOME_Z     = -0.20;  // Below frame plane
const float HOME_PITCH =  0.0;
const float HOME_ROLL  =  0.0;

// --- Friction calibration --------------------------------------------------
const int16_t  FRIC_CURRENT_STEP   = 2;     // Increment per iteration (Dxl units)
const int16_t  FRIC_CURRENT_MAX    = 80;    // Give up above this
const uint32_t FRIC_STEP_DELAY_MS  = 300;   // Wait per step
const int32_t  FRIC_VEL_THRESHOLD  = 3;     // Velocity units = "motor is moving"

// --- Tilt management -------------------------------------------------------
//  Hard-reject moves where the wrench residual predicts unacceptable tilt.
//  0.05 Nm ≈ 2–3° on a 150 g platform.  Raise if too restrictive, but the
//  EE WILL tilt at off-centre poses — this just tells you by how much.
const float MAX_RESIDUAL_MOMENT_NM = 0.05f;

// --- EE centre-of-mass vertical offset (metres) ----------------------------
//  Distance the CoM sits BELOW the cable attachment plane.
//  0 = CoM exactly at the attachment centre; positive = below (stabilising).
//  Used to include the gravity moment when pitch / roll ≠ 0.
//  Measure or estimate from a CAD model.
const float EE_COM_OFFSET_Z = 0.0f;

// --- Workspace limits (metres) — moves outside are rejected ----------------
//  Keep at least ~30 mm clearance inside the anchor footprint.
const float WS_X_MAX =  0.13f;   // ±  (FRAME_WIDTH/2  − margin)
const float WS_Y_MAX =  0.13f;   // ±  (FRAME_HEIGHT/2 − margin)
const float WS_Z_MIN = -0.35f;   // lower bound (below frame)
const float WS_Z_MAX = -0.05f;   // upper bound (stay below anchors)

// --- IMU orientation feedback ----------------------------------------------
//  The OpenCR 1.0 board has an onboard MPU9250.  Set this to 1 to enable
//  closed-loop orientation control using the measured roll / pitch.
//  Requires the OpenCR board's cIMU library.  When enabled the solver uses
//  the *actual* EE orientation so it corrects tilt rather than ignoring it.
#define USE_IMU  0   // 0 = open-loop (default), 1 = closed-loop via IMU

// ████████████████████████████████████████████████████████████████████████████
//  SECTION 2 — HARDWARE WIRING
// ████████████████████████████████████████████████████████████████████████████

#define DXL_SERIAL Serial3
const int DXL_DIR_PIN = 84;

const uint8_t NUM_MOTORS = 4;
const uint8_t DXL_ID[NUM_MOTORS] = {1, 3, 4, 2};   // Match your wiring

// Motor direction sign:  +1 = positive steps reel OUT,  -1 = positive steps reel IN.
// Flip if a motor winds the wrong way.
const int8_t MOTOR_DIR[NUM_MOTORS] = {1, 1, 1, 1};

const float DXL_PROTOCOL = 2.0;

// ████████████████████████████████████████████████████████████████████████████
//  SECTION 3 — ANCHOR & EE ATTACHMENT GEOMETRY
// ████████████████████████████████████████████████████████████████████████████

//  Anchor layout (top view, frame plane Z = 0):
//     A0 (+X,+Y)      A1 (-X,+Y)
//     A3 (+X,-Y)      A2 (-X,-Y)
const float ANCHOR[4][3] = {
  { FRAME_WIDTH / 2,  FRAME_HEIGHT / 2, 0},
  {-FRAME_WIDTH / 2,  FRAME_HEIGHT / 2, 0},
  {-FRAME_WIDTH / 2, -FRAME_HEIGHT / 2, 0},
  { FRAME_WIDTH / 2, -FRAME_HEIGHT / 2, 0}
};

// EE attachment points in the local body frame (same corner order)
const float EE_LOCAL[4][3] = {
  { EE_WIDTH / 2,  EE_LENGTH / 2, 0},
  {-EE_WIDTH / 2,  EE_LENGTH / 2, 0},
  {-EE_WIDTH / 2, -EE_LENGTH / 2, 0},
  { EE_WIDTH / 2, -EE_LENGTH / 2, 0}
};

// ████████████████████████████████████████████████████████████████████████████
//  SECTION 4 — GLOBAL STATE
// ████████████████████████████████████████████████████████████████████████████

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

int32_t motor_offset[NUM_MOTORS]     = {0, 0, 0, 0};
int16_t friction_current[NUM_MOTORS] = {0, 0, 0, 0};  // Coulomb friction (Dxl units, always ≥ 0)
bool    is_calibrating = false;
float   last_tensions[4] = {0};   // Most recent commanded tensions (for telemetry)
float   last_pretension_shift = 0; // Redistribution shift applied during last move (N)

// ████████████████████████████████████████████████████████████████████████████
//  SECTION 5 — MATH HELPERS
// ████████████████████████████████████████████████████████████████████████████

static inline void cross3(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1]*b[2] - a[2]*b[1];
  out[1] = a[2]*b[0] - a[0]*b[2];
  out[2] = a[0]*b[1] - a[1]*b[0];
}

// Gauss-Jordan 4×4 inverse with partial pivoting.
// Returns false if singular.
bool invert4x4(float A[4][4], float Ainv[4][4]) {
  float aug[4][8];
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      aug[i][j]   = A[i][j];
      aug[i][j+4] = (i == j) ? 1.0f : 0.0f;
    }

  for (int col = 0; col < 4; col++) {
    // Partial pivot
    int best = col;
    for (int r = col + 1; r < 4; r++)
      if (fabsf(aug[r][col]) > fabsf(aug[best][col])) best = r;
    if (best != col)
      for (int j = 0; j < 8; j++) {
        float tmp = aug[col][j]; aug[col][j] = aug[best][j]; aug[best][j] = tmp;
      }
    if (fabsf(aug[col][col]) < 1e-10f) return false;

    // Scale row
    float piv = aug[col][col];
    for (int j = 0; j < 8; j++) aug[col][j] /= piv;

    // Eliminate
    for (int r = 0; r < 4; r++) {
      if (r == col) continue;
      float f = aug[r][col];
      for (int j = 0; j < 8; j++) aug[r][j] -= f * aug[col][j];
    }
  }

  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      Ainv[i][j] = aug[i][j+4];
  return true;
}

// ████████████████████████████████████████████████████████████████████████████
//  SECTION 6 — INVERSE KINEMATICS
// ████████████████████████████████████████████████████████████████████████████
//
//  Given a desired EE pose (x, y, z, pitch, roll), computes for each cable:
//    lengths[i] — geometric cable length (m) including knot offset
//    u[i][3]    — unit vector from EE attachment toward anchor
//    r[i][3]    — vector from EE centre to attachment point (global frame)
//
void computeIK(float x, float y, float z, float pitch, float roll,
               float lengths[4], float u[4][3], float r[4][3]) {

  float cp = cosf(pitch), sp = sinf(pitch);
  float cr = cosf(roll),  sr = sinf(roll);

  // Rotation matrix R = Ry(pitch) · Rx(roll)
  //  | cp      sr*sp    cr*sp |
  //  | 0       cr      -sr    |
  //  |-sp      sr*cp    cr*cp |
  //  (acts on column vectors in body frame)

  for (int i = 0; i < 4; i++) {
    float lx = EE_LOCAL[i][0];
    float ly = EE_LOCAL[i][1];
    float lz = EE_LOCAL[i][2];   // usually 0

    // r_i = R * EE_LOCAL[i]  (attachment offset in global frame)
    r[i][0] =  cp * lx + sr*sp * ly + cr*sp * lz;
    r[i][1] =             cr   * ly - sr    * lz;
    r[i][2] = -sp * lx + sr*cp * ly + cr*cp * lz;

    // Global position of attachment point
    float px = x + r[i][0];
    float py = y + r[i][1];
    float pz = z + r[i][2];

    // Vector from attachment to anchor
    float dx = ANCHOR[i][0] - px;
    float dy = ANCHOR[i][1] - py;
    float dz = ANCHOR[i][2] - pz;

    float L = sqrtf(dx*dx + dy*dy + dz*dz);

    // Add knot offset (knot eats cable → need more cable paid out)
    lengths[i] = L + KNOT_OFFSET[i];

    // Unit direction (attachment → anchor)
    if (L > 1e-6f) {
      u[i][0] = dx / L;
      u[i][1] = dy / L;
      u[i][2] = dz / L;
    } else {
      u[i][0] = u[i][1] = u[i][2] = 0;
    }
  }
}

// Cable length → Dynamixel encoder steps
int32_t lengthToSteps(float length_m) {
  float revs = length_m / (2.0f * (float)M_PI * SPOOL_RADIUS);
  return (int32_t)(revs * DXL_STEPS_PER_REV);
}

// ████████████████████████████████████████████████████████████████████████████
//  SECTION 7 — 6-DOF WRENCH SOLVER
// ████████████████████████████████████████████████████████████████████████████
//
//  For a suspended CDPR with 4 cables controlling 6 DOF the system is
//  over-determined (6 eqns, 4 unknowns).  We find the least-squares
//  tension vector that best satisfies both FORCE and MOMENT balance,
//  then shift all tensions so every cable meets T_MIN_N.
//
//  This is the key improvement: the old code ignored moments entirely,
//  leading to large tilt whenever the EE moved off-centre.
//
//  Wrench equation:   W · T = b
//    W  (6×4):  column i = [ u_i ;  r_i × u_i ]
//    b  (6×1):  [ 0, 0, m·g, 0, 0, 0 ]
//               (gravity acts at CoM → no external moment about CoM)
//
//  Least-squares:  T = (WᵀW)⁻¹ Wᵀ b   →  4×4 inverse (Section 5)
//
//  Returns true if all tensions ≥ T_MIN_N after redistribution.
//  tensions_out[4] receives the final per-cable tensions.
//  residual_moment[3] receives the residual Mx, My, Mz that the
//    solver could NOT eliminate (tells you expected tilt magnitude).
//
bool solveTensions(float x, float y, float z, float pitch, float roll,
                   float tensions_out[4], float residual_moment[3],
                   float lengths_out[4]) {

  float u[4][3], r_vec[4][3];
  computeIK(x, y, z, pitch, roll, lengths_out, u, r_vec);

  // --- Build W (6×4) -------------------------------------------------------
  float W[6][4];
  for (int i = 0; i < 4; i++) {
    // Force rows
    W[0][i] = u[i][0];
    W[1][i] = u[i][1];
    W[2][i] = u[i][2];
    // Moment rows: r_i × u_i
    float rxu[3];
    cross3(r_vec[i], u[i], rxu);
    W[3][i] = rxu[0];
    W[4][i] = rxu[1];
    W[5][i] = rxu[2];
  }

  // --- b = external wrench (gravity only) ----------------------------------
  //  When EE_COM_OFFSET_Z > 0 the CoM is below the attachment plane.
  //  A tilted EE then has a gravity moment: M = r_com × F_grav.
  //  r_com (world) = R * [0, 0, -EE_COM_OFFSET_Z]  →
  //    Mx_cables =  m·g · sin(roll)        · EE_COM_OFFSET_Z
  //    My_cables =  m·g · cos(roll)·sin(pitch) · EE_COM_OFFSET_Z
  float cp = cosf(pitch), sp = sinf(pitch);
  float cr = cosf(roll),  sr = sinf(roll);
  float b[6] = {0, 0, ee_mass_kg * GRAVITY, 0, 0, 0};
  if (EE_COM_OFFSET_Z != 0.0f) {
    b[3] += ee_mass_kg * GRAVITY * sr        * EE_COM_OFFSET_Z;
    b[4] += ee_mass_kg * GRAVITY * cr * sp   * EE_COM_OFFSET_Z;
  }

  // --- WᵀW  (4×4) ---------------------------------------------------------
  float WtW[4][4] = {{0}};
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      for (int k = 0; k < 6; k++)
        WtW[i][j] += W[k][i] * W[k][j];

  // --- Wᵀb  (4×1) ---------------------------------------------------------
  float Wtb[4] = {0};
  for (int i = 0; i < 4; i++)
    for (int k = 0; k < 6; k++)
      Wtb[i] += W[k][i] * b[k];

  // --- Invert WᵀW ---------------------------------------------------------
  float WtW_inv[4][4];
  if (!invert4x4(WtW, WtW_inv)) {
    // Degenerate — cables nearly coplanar
    for (int i = 0; i < 4; i++) tensions_out[i] = -1;
    residual_moment[0] = residual_moment[1] = residual_moment[2] = 999;
    return false;
  }

  // --- T = (WᵀW)⁻¹ · Wᵀb -------------------------------------------------
  for (int i = 0; i < 4; i++) {
    tensions_out[i] = 0;
    for (int j = 0; j < 4; j++)
      tensions_out[i] += WtW_inv[i][j] * Wtb[j];
  }

  // --- Positive-tension redistribution -------------------------------------
  //  Shift all tensions upward so the smallest one = T_MIN_N.
  //  NOTE: a uniform shift of λ N adds extra wrench W·[λ,λ,λ,λ]ᵀ which
  //  includes a small moment when cables are asymmetric (off-centre poses).
  //  This is the dominant source of residual tilt — keep T_MIN_N small to
  //  minimise it, and/or enlarge the EE platform to increase moment authority.
  float t_min_found = tensions_out[0];
  for (int i = 1; i < 4; i++)
    if (tensions_out[i] < t_min_found) t_min_found = tensions_out[i];

  last_pretension_shift = 0;
  if (t_min_found < T_MIN_N) {
    last_pretension_shift = T_MIN_N - t_min_found;
    for (int i = 0; i < 4; i++)
      tensions_out[i] += last_pretension_shift;
  }

  // --- Compute residual wrench  (W·T - b) ----------------------------------
  //  The force residual is typically tiny (over-determined LS minimises it).
  //  The moment residual shows how much tilt to expect.
  float residual[6] = {0};
  for (int k = 0; k < 6; k++) {
    for (int i = 0; i < 4; i++)
      residual[k] += W[k][i] * tensions_out[i];
    residual[k] -= b[k];
  }
  residual_moment[0] = residual[3];  // Mx
  residual_moment[1] = residual[4];  // My
  residual_moment[2] = residual[5];  // Mz

  return true;
}

// ████████████████████████████████████████████████████████████████████████████
//  SECTION 8 — TENSION → CURRENT CONVERSION
// ████████████████████████████████████████████████████████████████████████████
//
//  Cable tension T (N) → output torque τ = T × R_spool
//  Motor current I (A)  = τ / Kt_eff
//  Dynamixel units      = I / DXL_CURRENT_UNIT
//
//  We add the per-motor Coulomb friction estimate so the motor can
//  overcome its own internal resistance before producing useful torque.
//

int16_t tensionToCurrent(float tension_N, int motor_idx) {
  float torque   = tension_N * SPOOL_RADIUS;           // Nm
  float current_A = torque / TORQUE_CONSTANT;           // A
  int16_t dxl_units = (int16_t)(current_A / DXL_CURRENT_UNIT);

  // Add friction compensation + safety margin
  dxl_units += friction_current[motor_idx];
  dxl_units += CURRENT_MARGIN;

  // Clamp
  if (dxl_units < 1)           dxl_units = 1;
  if (dxl_units > CURRENT_CAP) dxl_units = CURRENT_CAP;

  return dxl_units;
}

// ████████████████████████████████████████████████████████████████████████████
//  SECTION 9 — MOVE TO TARGET
// ████████████████████████████████████████████████████████████████████████████

void moveToTarget(float x, float y, float z, float pitch, float roll) {

  // --- 0. Workspace bounds check ------------------------------------------
  if (fabsf(x) > WS_X_MAX || fabsf(y) > WS_Y_MAX ||
      z < WS_Z_MIN         || z > WS_Z_MAX) {
    Serial.println(F("ERROR: Target outside workspace limits. Move rejected."));
    Serial.print(F("  Limits: |X|<")); Serial.print(WS_X_MAX,3);
    Serial.print(F("  |Y|<")); Serial.print(WS_Y_MAX,3);
    Serial.print(F("  Z in [")); Serial.print(WS_Z_MIN,3);
    Serial.print(F(",")); Serial.print(WS_Z_MAX,3); Serial.println(F("]"));
    return;
  }

  // --- 1. Read actual orientation from IMU (if enabled) -------------------
#if USE_IMU
  //  The IMU gives the real EE roll & pitch.  Solving for the *actual*
  //  orientation means tensions are correct for where the EE really is,
  //  creating a restoring wrench back toward commanded zero tilt.
  float solve_pitch = imu.getPitchAngle() * DEG_TO_RAD;
  float solve_roll  = imu.getRollAngle()  * DEG_TO_RAD;
#else
  float solve_pitch = pitch;
  float solve_roll  = roll;
#endif

  // --- 2. Solve optimal tensions -------------------------------------------
  float tensions[4];
  float res_moment[3];
  float lengths[4];
  bool ok = solveTensions(x, y, z, solve_pitch, solve_roll,
                          tensions, res_moment, lengths);

  if (!ok) {
    Serial.println(F("ERROR: Degenerate cable configuration. Move rejected."));
    return;
  }

  // --- 3. Report tensions, shift & residual --------------------------------
  Serial.print(F("  Tensions (N):  "));
  for (int i = 0; i < 4; i++) {
    Serial.print(tensions[i], 3); Serial.print(F("  "));
  }
  Serial.println();

  if (last_pretension_shift > 0) {
    Serial.print(F("  Pre-tension shift: "));
    Serial.print(last_pretension_shift, 3);
    Serial.println(F(" N  (adds uncorrectable moment at off-centre poses)"));
  }

  Serial.print(F("  Residual M (Nm): Mx="));
  Serial.print(res_moment[0], 5); Serial.print(F("  My="));
  Serial.print(res_moment[1], 5); Serial.print(F("  Mz="));
  Serial.println(res_moment[2], 5);

  float moment_mag = sqrtf(res_moment[0]*res_moment[0]
                         + res_moment[1]*res_moment[1]
                         + res_moment[2]*res_moment[2]);

  if (moment_mag > MAX_RESIDUAL_MOMENT_NM) {
    Serial.print(F("ERROR: |M_residual| = "));
    Serial.print(moment_mag, 4);
    Serial.print(F(" Nm exceeds limit (")); Serial.print(MAX_RESIDUAL_MOMENT_NM, 3);
    Serial.println(F(" Nm). Move rejected to prevent tilt."));
    Serial.println(F("  → Try a smaller offset, larger EE footprint, or raise MAX_RESIDUAL_MOMENT_NM."));
    return;
  } else if (moment_mag > 0.01f) {
    Serial.print(F("  WARNING: |M_residual| = "));
    Serial.print(moment_mag, 4);
    Serial.println(F(" Nm — expect minor tilt at this pose."));
  }

  // --- 4. Command each motor: position + current --------------------------
  //  Cable lengths already computed by solveTensions — no second IK call needed.
  Serial.print(F("  CMD →  "));
  for (int i = 0; i < 4; i++) {
    // Position: geometric length → steps → apply calibration offset & direction
    int32_t steps = lengthToSteps(lengths[i]);
    int32_t cmd_pos = MOTOR_DIR[i] * steps + motor_offset[i];

    // Current: tension → Dynamixel current units (with friction comp.)
    int16_t cmd_cur = tensionToCurrent(tensions[i], i);

    // Send to motor (set current first, then position)
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

// ████████████████████████████████████████████████████████████████████████████
//  SECTION 10 — HOME CALIBRATION
// ████████████████████████████████████████████████████████████████████████████
//
//  1. 'C' → motors switch to pure current mode and reel in gently.
//           You manually guide the EE to the home position.
//  2. 'S' → controller reads encoder positions, computes the offset
//           between measured and theoretical steps.
//

void startCalibration() {
  is_calibrating = true;
  Serial.println(F("--- HOME CALIBRATION: constant-tension mode ---"));
  Serial.println(F("Guide the end-effector to the home position, then send 'S'."));

  for (int i = 0; i < NUM_MOTORS; i++) {
    dxl.torqueOff(DXL_ID[i]);
    dxl.setOperatingMode(DXL_ID[i], OP_CURRENT);
    dxl.torqueOn(DXL_ID[i]);
    dxl.setGoalCurrent(DXL_ID[i], CAL_TENSION_CURRENT);
  }
}

void stopCalibration() {
  if (!is_calibrating) { Serial.println(F("Not calibrating.")); return; }

  Serial.println(F("--- REGISTERING HOME ---"));

  float home_len[4], home_u[4][3], home_r[4][3];
  computeIK(HOME_X, HOME_Y, HOME_Z, HOME_PITCH, HOME_ROLL,
            home_len, home_u, home_r);

  for (int i = 0; i < NUM_MOTORS; i++) {
    int32_t theo = MOTOR_DIR[i] * lengthToSteps(home_len[i]);
    int32_t raw  = dxl.getPresentPosition(DXL_ID[i]);
    motor_offset[i] = raw - theo;

    // Switch back to Mode 5
    dxl.torqueOff(DXL_ID[i]);
    dxl.setOperatingMode(DXL_ID[i], OP_CURRENT_BASED_POSITION);
    dxl.writeControlTableItem(PROFILE_VELOCITY,    DXL_ID[i], PROFILE_VEL);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID[i], PROFILE_ACC);
    dxl.writeControlTableItem(GOAL_CURRENT, DXL_ID[i], CURRENT_CAP);
    dxl.torqueOn(DXL_ID[i]);
    dxl.setGoalPosition(DXL_ID[i], raw);   // Hold in place

    Serial.print(F("  M")); Serial.print(DXL_ID[i]);
    Serial.print(F(" | raw=")); Serial.print(raw);
    Serial.print(F(" | theo=")); Serial.print(theo);
    Serial.print(F(" | offset=")); Serial.println(motor_offset[i]);
  }

  is_calibrating = false;
  Serial.println(F("Home calibration complete."));
}

// ████████████████████████████████████████████████████████████████████████████
//  SECTION 11 — FRICTION IDENTIFICATION
// ████████████████████████████████████████████████████████████████████████████
//
//  For each motor (one at a time):
//    • Switch to current control mode
//    • Ramp current from 0 upward in small steps
//    • Monitor present velocity
//    • When |velocity| exceeds threshold → that current = Coulomb friction
//    • Test both directions, average
//    • Store in friction_current[i]
//
//  This measures the combined motor + gearbox + spool static friction.
//  At the low tensions in this system (~0.5 N), friction is a dominant
//  error source, so this calibration matters a lot.
//

void runFrictionCalibration() {
  Serial.println(F("=== FRICTION IDENTIFICATION ==="));
  Serial.println(F("Keep the end-effector stationary. Motors will twitch briefly."));

  for (int m = 0; m < NUM_MOTORS; m++) {
    Serial.print(F("  Motor ")); Serial.print(DXL_ID[m]); Serial.print(F(" ... "));

    int16_t threshold_pos = 0;
    int16_t threshold_neg = 0;

    // --- Save current state and switch to current mode ---
    int32_t saved_pos = dxl.getPresentPosition(DXL_ID[m]);
    dxl.torqueOff(DXL_ID[m]);
    dxl.setOperatingMode(DXL_ID[m], OP_CURRENT);
    dxl.torqueOn(DXL_ID[m]);

    // --- Positive direction ramp ---
    for (int16_t c = FRIC_CURRENT_STEP; c <= FRIC_CURRENT_MAX; c += FRIC_CURRENT_STEP) {
      dxl.setGoalCurrent(DXL_ID[m], c);
      delay(FRIC_STEP_DELAY_MS);
      int32_t vel = dxl.getPresentVelocity(DXL_ID[m]);
      if (abs(vel) >= FRIC_VEL_THRESHOLD) {
        threshold_pos = c;
        break;
      }
    }
    dxl.setGoalCurrent(DXL_ID[m], 0);
    delay(200);

    // --- Negative direction ramp ---
    for (int16_t c = -FRIC_CURRENT_STEP; c >= -FRIC_CURRENT_MAX; c -= FRIC_CURRENT_STEP) {
      dxl.setGoalCurrent(DXL_ID[m], c);
      delay(FRIC_STEP_DELAY_MS);
      int32_t vel = dxl.getPresentVelocity(DXL_ID[m]);
      if (abs(vel) >= FRIC_VEL_THRESHOLD) {
        threshold_neg = abs(c);
        break;
      }
    }
    dxl.setGoalCurrent(DXL_ID[m], 0);
    delay(200);

    // Average of both directions
    if (threshold_pos == 0 && threshold_neg == 0) {
      friction_current[m] = 5;  // Default fallback
      Serial.println(F("could not detect — using default (5)"));
    } else if (threshold_pos == 0) {
      friction_current[m] = threshold_neg;
    } else if (threshold_neg == 0) {
      friction_current[m] = threshold_pos;
    } else {
      friction_current[m] = (threshold_pos + threshold_neg) / 2;
    }

    Serial.print(F("friction = "));
    Serial.print(friction_current[m]);
    Serial.print(F(" units (≈ "));
    Serial.print(friction_current[m] * DXL_CURRENT_UNIT * 1000.0f, 1);
    Serial.println(F(" mA)"));

    // --- Restore Mode 5 and return to saved position ---
    dxl.torqueOff(DXL_ID[m]);
    dxl.setOperatingMode(DXL_ID[m], OP_CURRENT_BASED_POSITION);
    dxl.writeControlTableItem(PROFILE_VELOCITY,    DXL_ID[m], PROFILE_VEL);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID[m], PROFILE_ACC);
    dxl.writeControlTableItem(GOAL_CURRENT, DXL_ID[m], CURRENT_CAP);
    dxl.torqueOn(DXL_ID[m]);
    dxl.setGoalPosition(DXL_ID[m], saved_pos);
    delay(500);
  }

  Serial.println(F("=== Friction calibration complete ==="));
}

// ████████████████████████████████████████████████████████████████████████████
//  SECTION 12 — WEIGHT ESTIMATION
// ████████████████████████████████████████████████████████████████████████████
//
//  At the current (known) pose, read the present current from each motor,
//  convert to cable tension, project onto vertical axis, and sum.
//  This gives the total suspended weight.
//
//  Best run after home calibration, with the EE at the home position.
//  Can be re-run after attaching a gripper or picking up an object.
//

void runWeightEstimation() {
  Serial.println(F("=== WEIGHT ESTIMATION ==="));
  Serial.println(F("Ensure the EE is stationary at a known pose (ideally home)."));
  Serial.println(F("Waiting for settle..."));
  delay(CAL_SETTLE_MS);

  // Use home pose for direction vectors
  float lengths[4], u[4][3], r_vec[4][3];
  computeIK(HOME_X, HOME_Y, HOME_Z, HOME_PITCH, HOME_ROLL,
            lengths, u, r_vec);

  float total_Fz = 0;

  for (int i = 0; i < NUM_MOTORS; i++) {
    // Read present current (signed, Dynamixel units)
    int16_t raw_current = dxl.getPresentCurrent(DXL_ID[i]);
    float current_A = fabsf((float)raw_current * DXL_CURRENT_UNIT);

    // Subtract estimated friction (friction doesn't contribute to useful tension)
    float friction_A = (float)friction_current[i] * DXL_CURRENT_UNIT;
    float useful_A = current_A - friction_A;
    if (useful_A < 0) useful_A = 0;

    // Convert to cable tension
    float torque = useful_A * TORQUE_CONSTANT;
    float tension = torque / SPOOL_RADIUS;

    // Project onto Z axis (vertical component)
    // u[i][2] is the Z component of the unit vector (attachment → anchor = upward)
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
    Serial.print(F(" g → ")); Serial.print(ee_mass_kg * 1000, 1);
    Serial.println(F(" g"));
  } else {
    Serial.println(F("  Estimated mass too low — keeping previous value."));
  }

  Serial.println(F("=== Weight estimation complete ==="));
}

// ████████████████████████████████████████████████████████████████████████████
//  SECTION 13 — DIAGNOSTICS
// ████████████████████████████████████████████████████████████████████████████

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
  for (int i = 0; i < 4; i++) {
    Serial.print(last_tensions[i], 3); Serial.print(F("  "));
  }
  Serial.println();
  Serial.print(F("  Pre-tension shift: ")); Serial.print(last_pretension_shift, 3);
  Serial.println(F(" N"));
  Serial.println(F("-------------------"));
}

void printConfig() {
  Serial.println(F("--- Configuration ---"));
  Serial.print(F("  Frame:        ")); Serial.print(FRAME_WIDTH*100,1);
  Serial.print(F(" × "));              Serial.print(FRAME_HEIGHT*100,1);
  Serial.println(F(" cm"));
  Serial.print(F("  End-effector: ")); Serial.print(EE_WIDTH*100,1);
  Serial.print(F(" × "));              Serial.print(EE_LENGTH*100,1);
  Serial.println(F(" cm"));
  Serial.print(F("  Spool Ø:      ")); Serial.print(SPOOL_RADIUS*1000,1);
  Serial.println(F(" mm"));
  Serial.print(F("  EE mass:      ")); Serial.print(ee_mass_kg*1000,0);
  Serial.println(F(" g"));
  Serial.print(F("  Kt_eff:       ")); Serial.print(TORQUE_CONSTANT,3);
  Serial.println(F(" Nm/A"));
  Serial.print(F("  Knot offsets:  "));
  for (int i = 0; i < 4; i++) {
    Serial.print(KNOT_OFFSET[i]*1000,1); Serial.print(F("mm  "));
  }
  Serial.println();
  Serial.print(F("  T_min:        ")); Serial.print(T_MIN_N,2);
  Serial.println(F(" N"));
  Serial.print(F("  Max resid M:  ")); Serial.print(MAX_RESIDUAL_MOMENT_NM,3);
  Serial.println(F(" Nm"));
  Serial.print(F("  CoM offset Z: ")); Serial.print(EE_COM_OFFSET_Z*1000,1);
  Serial.println(F(" mm"));
  Serial.print(F("  Workspace X:  ±")); Serial.print(WS_X_MAX*100,1); Serial.println(F(" cm"));
  Serial.print(F("  Workspace Y:  ±")); Serial.print(WS_Y_MAX*100,1); Serial.println(F(" cm"));
  Serial.print(F("  Workspace Z:  [")); Serial.print(WS_Z_MIN*100,1);
  Serial.print(F(", ")); Serial.print(WS_Z_MAX*100,1); Serial.println(F("] cm"));
#if USE_IMU
  Serial.println(F("  IMU feedback: ENABLED"));
#else
  Serial.println(F("  IMU feedback: disabled (set USE_IMU 1 to enable)"));
#endif
  Serial.print(F("  Home: "));
  Serial.print(HOME_X,3); Serial.print(F(", "));
  Serial.print(HOME_Y,3); Serial.print(F(", "));
  Serial.print(HOME_Z,3); Serial.print(F(", p="));
  Serial.print(HOME_PITCH,3); Serial.print(F(", r="));
  Serial.println(HOME_ROLL,3);
  Serial.println(F("---------------------"));
}

void printHelp() {
  Serial.println(F("=== CDPR Controller v2.0 ==="));
  Serial.println(F("Commands:"));
  Serial.println(F("  C           Start home calibration (constant tension)"));
  Serial.println(F("  S           Stop calibration & register home"));
  Serial.println(F("  F           Run friction identification"));
  Serial.println(F("  W           Run weight estimation"));
  Serial.println(F("  D           Print diagnostics (positions, currents)"));
  Serial.println(F("  I           Print configuration"));
  Serial.println(F("  K i val     Set knot offset: K 0 3.5  → cable 0 = 3.5 mm"));
  Serial.println(F("  M val       Set mass manually: M 250  → 250 g"));
  Serial.println(F("  X,Y,Z,P,R  Move to pose  (e.g. 0.0,0.0,-0.2,0.0,0.0)"));
  Serial.println(F("  H           Move to home position"));
  Serial.println(F("  ?           Print this help"));
  Serial.println();
}

// ████████████████████████████████████████████████████████████████████████████
//  SECTION 14 — SETUP
// ████████████████████████████████████████████████████████████████████████████

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);  // Wait up to 3s for serial monitor

#if USE_IMU
  imu.begin();
  delay(500);  // Let IMU settle
  Serial.println(F("IMU orientation feedback ENABLED."));
#else
  Serial.println(F("IMU disabled (USE_IMU=0). Open-loop orientation."));
#endif

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
    dxl.writeControlTableItem(PROFILE_VELOCITY,    DXL_ID[i], PROFILE_VEL);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID[i], PROFILE_ACC);
    dxl.writeControlTableItem(GOAL_CURRENT, DXL_ID[i], CURRENT_CAP);
    dxl.torqueOn(DXL_ID[i]);
  }

  if (!all_ok) {
    Serial.println(F("WARNING: Not all motors responded. Check wiring."));
  }

  printHelp();
  printConfig();
}

// ████████████████████████████████████████████████████████████████████████████
//  SECTION 15 — MAIN LOOP (command parser)
// ████████████████████████████████████████████████████████████████████████████

void loop() {
#if USE_IMU
  imu.update();  // Refresh Mahony/Madgwick filter — call every iteration
#endif

  if (Serial.available() <= 0) return;

  char first = Serial.peek();

  // --- Single-character commands ---
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
    if (is_calibrating) { Serial.println(F("Finish home cal first.")); return; }
    runFrictionCalibration();
    return;
  }
  if (first == 'W' || first == 'w') {
    Serial.readStringUntil('\n');
    if (is_calibrating) { Serial.println(F("Finish home cal first.")); return; }
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
    if (is_calibrating) { Serial.println(F("Finish home cal first.")); return; }
    Serial.println(F("Moving to home..."));
    moveToTarget(HOME_X, HOME_Y, HOME_Z, HOME_PITCH, HOME_ROLL);
    return;
  }

  // --- Knot offset:  K <index> <mm> ---
  if (first == 'K' || first == 'k') {
    Serial.read();  // consume 'K'
    int idx = Serial.parseInt();
    float mm = Serial.parseFloat();
    Serial.readStringUntil('\n');
    if (idx >= 0 && idx < 4) {
      KNOT_OFFSET[idx] = mm / 1000.0f;
      Serial.print(F("Knot offset[")); Serial.print(idx);
      Serial.print(F("] = ")); Serial.print(mm, 2);
      Serial.println(F(" mm"));
    } else {
      Serial.println(F("Invalid index (0-3)."));
    }
    return;
  }

  // --- Manual mass:  M <grams> ---
  if (first == 'M' || first == 'm') {
    Serial.read();  // consume 'M'
    float g = Serial.parseFloat();
    Serial.readStringUntil('\n');
    if (g > 0) {
      ee_mass_kg = g / 1000.0f;
      Serial.print(F("Mass set to ")); Serial.print(g, 1); Serial.println(F(" g"));
    } else {
      Serial.println(F("Invalid mass."));
    }
    return;
  }

  // --- Pose:  X,Y,Z,P,R ---
  if (is_calibrating) {
    Serial.readStringUntil('\n');
    Serial.println(F("Calibrating — send 'S' to finish first."));
    return;
  }

  // Try to parse as numeric pose
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

  // Unknown command
  Serial.readStringUntil('\n');
  Serial.println(F("Unknown command. Send '?' for help."));
}
