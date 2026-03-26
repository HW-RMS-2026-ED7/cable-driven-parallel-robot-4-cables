// ============================================================================
//  cdpr_kinematics.cpp — Inverse kinematics & 6-DOF wrench solver
// ============================================================================

#include "cdpr_kinematics.h"
#include "config.h"
#include <math.h>

// ─── Math helpers ───────────────────────────────────────────────────────────

static inline void cross3(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1]*b[2] - a[2]*b[1];
  out[1] = a[2]*b[0] - a[0]*b[2];
  out[2] = a[0]*b[1] - a[1]*b[0];
}

// Gauss-Jordan 4x4 inverse (double precision internally)
static bool invert4x4(float A[4][4], float Ainv[4][4]) {
  double aug[4][8];
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      aug[i][j]   = (double)A[i][j];
      aug[i][j+4] = (i == j) ? 1.0 : 0.0;
    }

  for (int col = 0; col < 4; col++) {
    int best = col;
    for (int r = col + 1; r < 4; r++)
      if (fabs(aug[r][col]) > fabs(aug[best][col])) best = r;
    if (best != col)
      for (int j = 0; j < 8; j++) {
        double tmp = aug[col][j]; aug[col][j] = aug[best][j]; aug[best][j] = tmp;
      }
    if (fabs(aug[col][col]) < 1e-15) return false;

    double piv = aug[col][col];
    for (int j = 0; j < 8; j++) aug[col][j] /= piv;

    for (int r = 0; r < 4; r++) {
      if (r == col) continue;
      double f = aug[r][col];
      for (int j = 0; j < 8; j++) aug[r][j] -= f * aug[col][j];
    }
  }

  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      Ainv[i][j] = (float)aug[i][j+4];
  return true;
}

// ─── Inverse Kinematics ────────────────────────────────────────────────────

void computeIK(float x, float y, float z, float pitch, float roll,
               float lengths[4], float u[4][3], float r[4][3]) {

  float cp = cosf(pitch), sp = sinf(pitch);
  float cr = cosf(roll),  sr = sinf(roll);

  for (int i = 0; i < 4; i++) {
    float lx = EE_LOCAL[i][0];
    float ly = EE_LOCAL[i][1];
    float lz = EE_LOCAL[i][2];

    // R = Ry(pitch) * Rx(roll)
    r[i][0] =  cp * lx + sr*sp * ly + cr*sp * lz;
    r[i][1] =             cr   * ly - sr    * lz;
    r[i][2] = -sp * lx + sr*cp * ly + cr*cp * lz;

    float px = x + r[i][0];
    float py = y + r[i][1];
    float pz = z + r[i][2];

    float dx = ANCHOR[i][0] - px;
    float dy = ANCHOR[i][1] - py;
    float dz = ANCHOR[i][2] - pz;

    float L = sqrtf(dx*dx + dy*dy + dz*dz);
    lengths[i] = L + KNOT_OFFSET[i];

    if (L > 1e-6f) {
      u[i][0] = dx / L;
      u[i][1] = dy / L;
      u[i][2] = dz / L;
    } else {
      u[i][0] = u[i][1] = u[i][2] = 0;
    }
  }
}

int32_t lengthToSteps(float length_m) {
  float revs = length_m / (2.0f * (float)M_PI * SPOOL_RADIUS);
  return (int32_t)(revs * DXL_STEPS_PER_REV);
}

// ─── 6-DOF Wrench Solver ───────────────────────────────────────────────────

bool solveTensions(float x, float y, float z, float pitch, float roll,
                   float tensions_out[4], float residual_moment[3]) {

  float lengths[4], u[4][3], r_vec[4][3];
  computeIK(x, y, z, pitch, roll, lengths, u, r_vec);

  // Build W (6x4)
  float W[6][4];
  for (int i = 0; i < 4; i++) {
    W[0][i] = u[i][0];
    W[1][i] = u[i][1];
    W[2][i] = u[i][2];
    float rxu[3];
    cross3(r_vec[i], u[i], rxu);
    W[3][i] = rxu[0];
    W[4][i] = rxu[1];
    W[5][i] = rxu[2];
  }

  float b[6] = {0, 0, ee_mass_kg * GRAVITY, 0, 0, 0};

  // WtW (4x4)
  float WtW[4][4] = {{0}};
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      for (int k = 0; k < 6; k++)
        WtW[i][j] += W[k][i] * W[k][j];

  // Wtb (4x1)
  float Wtb[4] = {0};
  for (int i = 0; i < 4; i++)
    for (int k = 0; k < 6; k++)
      Wtb[i] += W[k][i] * b[k];

  // Tikhonov regularization: prevent singularity at symmetric poses
  for (int i = 0; i < 4; i++)
    WtW[i][i] += 1e-4f;

  // Invert
  float WtW_inv[4][4];
  if (!invert4x4(WtW, WtW_inv)) {
    for (int i = 0; i < 4; i++) tensions_out[i] = -1;
    residual_moment[0] = residual_moment[1] = residual_moment[2] = 999;
    return false;
  }

  // T = (WtW)^-1 * Wtb
  for (int i = 0; i < 4; i++) {
    tensions_out[i] = 0;
    for (int j = 0; j < 4; j++)
      tensions_out[i] += WtW_inv[i][j] * Wtb[j];
  }

  // Positive-tension redistribution
  float t_min_found = tensions_out[0];
  for (int i = 1; i < 4; i++)
    if (tensions_out[i] < t_min_found) t_min_found = tensions_out[i];

  if (t_min_found < T_MIN_N) {
    float shift = T_MIN_N - t_min_found;
    for (int i = 0; i < 4; i++)
      tensions_out[i] += shift;
  }

  // Residual wrench (W*T - b)
  float residual[6] = {0};
  for (int k = 0; k < 6; k++) {
    for (int i = 0; i < 4; i++)
      residual[k] += W[k][i] * tensions_out[i];
    residual[k] -= b[k];
  }
  residual_moment[0] = residual[3];
  residual_moment[1] = residual[4];
  residual_moment[2] = residual[5];

  return true;
}
