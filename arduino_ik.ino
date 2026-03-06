#include <Dynamixel2Arduino.h>
#include <math.h> // Standard C math library for sin, cos, sqrt

// --- Configuration OpenCR & Dynamixel ---
#define DXL_SERIAL   Serial3
const int DXL_DIR_PIN = 84;
const uint8_t DXL_ID[4] = {1, 2, 3, 4};
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

// --- Robot Kinematics Parameters ---
const float S = 0.4;  // m - Outside frame size
const float E_W = 0.1;  // m - End effector width
const float E_L = 0.1;  // m - End effector length
const float SPOOL_RADIUS = 0.06; // m - End effector height
const float DXL_STEPS_PER_REV = 4096.0;

const float A[4][3] = {
  {S/2, S/2, 0}, {-S/2, S/2, 0}, {-S/2, -S/2, 0}, {S/2, -S/2, 0}
};

const float E_local[4][3] = {
  {E_W/2, E_L/2, 0}, {-E_W/2, E_L/2, 0}, {-E_W/2, -E_L/2, 0}, {E_W/2, -E_L/2, 0}
};

// --- Calibration Variables ---
bool is_calibrating = false;
int32_t motor_offsets[4] = {0, 0, 0, 0}; 

// Define your physical Home position (Where you hold it during calibration)
const float HOME_X = 0.0;
const float HOME_Y = 0.0;
const float HOME_Z = -0.5; // Adjust this to your preferred default height
const float HOME_PITCH = 0.0;
const float HOME_ROLL = 0.0;

// The amount of current (tension) to apply during calibration (0 - 1193)
// XM430 uses ~2.69mA per unit. 30 is a very light pull.
const int16_t TENSION_CURRENT = -30;

void setup() {
  Serial.begin(115200);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for(int i = 0; i < 4; i++) {
    dxl.ping(DXL_ID[i]);
    dxl.torqueOff(DXL_ID[i]);
    dxl.setOperatingMode(DXL_ID[i], OP_EXTENDED_POSITION);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[i], 0); 
    dxl.torqueOn(DXL_ID[i]);
  }
  
  Serial.println("System Ready!");
  Serial.println("Commands:");
  Serial.println(" 'C' -> Enter Calibration Mode (Constant Tension)");
  Serial.println(" 'S' -> Stop Calibration & Register Home");
  Serial.println(" Or send coordinates: X, Y, Z, Pitch, Roll");
}

void loop() {
  if (Serial.available() > 0) {
    char first_char = Serial.peek();
    
    if (first_char == 'C' || first_char == 'c') {
      Serial.readStringUntil('\n'); // Clear buffer
      startCalibrationMode();
    } 
    else if (first_char == 'S' || first_char == 's') {
      Serial.readStringUntil('\n'); // Clear buffer
      stopAndRegisterCalibration();
    } 
    else if (!is_calibrating) {
      // Parse coordinates only if we are NOT calibrating
      float target_x = Serial.parseFloat();
      float target_y = Serial.parseFloat();
      float target_z = Serial.parseFloat();
      float target_pitch = Serial.parseFloat();
      float target_roll = Serial.parseFloat();
      Serial.readStringUntil('\n'); // Clear buffer
      
      calculateAndMove(target_x, target_y, target_z, target_pitch, target_roll);
    } 
    else {
      // If user sends numbers during calibration, flush them
      Serial.readStringUntil('\n');
      Serial.println("Error: System is calibrating. Send 'S' to stop first.");
    }
  }
}

void startCalibrationMode() {
  is_calibrating = true;
  Serial.println("--- ENTERING CALIBRATION MODE ---");
  Serial.println("Motors are in tension. Move effector to physical home position.");
  
  for(int i = 0; i < 4; i++) {
    dxl.torqueOff(DXL_ID[i]);
    dxl.setOperatingMode(DXL_ID[i], OP_CURRENT);
    dxl.torqueOn(DXL_ID[i]);
    
    // Command a light pull. If motors pull the wrong way, make this -TENSION_CURRENT
    dxl.setGoalCurrent(DXL_ID[i], TENSION_CURRENT); 
  }
}

void stopAndRegisterCalibration() {
  if (!is_calibrating) {
    Serial.println("Not currently calibrating.");
    return;
  }
  
  Serial.println("--- REGISTERING HOME POSITION ---");
  
  // Calculate theoretical step counts at the Home position
  int32_t theoretical_home_steps[4];
  calculateIKSteps(HOME_X, HOME_Y, HOME_Z, HOME_PITCH, HOME_ROLL, theoretical_home_steps);

  for(int i = 0; i < 4; i++) {
    // 1. Read actual physical position from the encoder
    int32_t actual_raw_steps = dxl.getPresentPosition(DXL_ID[i]);
    
    // 2. Calculate the offset mapping the physical world to the math model
    // Equation: Offset = Actual Raw - Theoretical Math
    motor_offsets[i] = actual_raw_steps - theoretical_home_steps[i];
    
    // 3. Switch back to extended position mode
    dxl.torqueOff(DXL_ID[i]);
    dxl.setOperatingMode(DXL_ID[i], OP_EXTENDED_POSITION);
    dxl.torqueOn(DXL_ID[i]);
    
    // Command motor to stay exactly where it is right now
    dxl.setGoalPosition(DXL_ID[i], actual_raw_steps);

    Serial.print("M"); Serial.print(DXL_ID[i]);
    Serial.print(" | Raw: "); Serial.print(actual_raw_steps);
    Serial.print(" | Math: "); Serial.print(theoretical_home_steps[i]);
    Serial.print(" | Offset Saved: "); Serial.println(motor_offsets[i]);
  }
  
  is_calibrating = false;
  Serial.println("Calibration complete. Ready for coordinates.");
}

// Extracted the core IK math into a reusable function
void calculateIKSteps(float x, float y, float z, float pitch, float roll, int32_t* steps_output) {
  float cp = cos(pitch); float sp = sin(pitch);
  float cr = cos(roll);  float sr = sin(roll);

  for(int i = 0; i < 4; i++) {
    float lx = E_local[i][0];
    float ly = E_local[i][1];

    float rot_x = cp * lx;
    float rot_y = (sr * sp) * lx + cr * ly;
    float rot_z = (-cr * sp) * lx + sr * ly;

    float gx = x + rot_x; float gy = y + rot_y; float gz = z + rot_z;
    float dx = A[i][0] - gx; float dy = A[i][1] - gy; float dz = A[i][2] - gz;
    
    float length = sqrt(dx*dx + dy*dy + dz*dz);
    steps_output[i] = (int32_t)((length / (2.0 * M_PI * SPOOL_RADIUS)) * DXL_STEPS_PER_REV);
  }
}

void calculateAndMove(float x, float y, float z, float pitch, float roll) {
  int32_t target_steps[4];
  calculateIKSteps(x, y, z, pitch, roll, target_steps);

  for(int i = 0; i < 4; i++) {
    // Add the calibration offset to the theoretical steps before sending to the motor
    int32_t final_command = target_steps[i] + motor_offsets[i];
    dxl.setGoalPosition(DXL_ID[i], final_command);
    
    Serial.print("M"); Serial.print(DXL_ID[i]); 
    Serial.print(" Target: "); Serial.print(final_command); Serial.print(" | ");
  }
  Serial.println();
}
