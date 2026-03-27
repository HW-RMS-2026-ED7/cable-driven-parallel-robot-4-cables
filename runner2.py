import mujoco
import mujoco.viewer
import numpy as np

# --- Import Kinematics ---
from kinematics import calculate_local_coordinates

# Frame parameters (make sure these match your XML generation if you separated them)
S = 2.0


def calculate_target_lengths(x, y, z, pitch, roll):
    """Calculates exactly how long the cables should be to hold this position."""
    local_coordinates = calculate_local_coordinates(pitch, roll)


# --- MuJoCo Setup ---
model = mujoco.MjModel.from_xml_path("cable_robot.xml")
data = mujoco.MjData(model)

# We calculate the lengths for a perfectly centered, level platform ONCE
TARGET_X, TARGET_Y, TARGET_Z = 0.0, 0.0, 1.0
TARGET_PITCH, TARGET_ROLL = 0.0, 0.0

static_target_L = calculate_target_lengths(
    TARGET_X, TARGET_Y, TARGET_Z, TARGET_PITCH, TARGET_ROLL
)


def my_controller(model, data):
    """Holds the platform center."""
    


mujoco.set_mjcb_control(my_controller)

print("Simulation started!")
print("--- INTERACTIVE CONTROLS ---")
print("1. Double-click the blue end-effector to select it.")
print("2. Hold 'Ctrl' and Right-Click-Drag to apply physical forces to it.")
print("3. Watch how it tilts and how the cables react!")

mujoco.viewer.launch(model, data)
mujoco.set_mjcb_control(None)
