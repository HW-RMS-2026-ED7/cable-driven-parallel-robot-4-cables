import mujoco
import mujoco.viewer
import math as m
import numpy as np

# --- Import Kinematics ---
from kinematics import (
    calculate_string_lengths,
    calculate_local_coordinates,
)


def calculate_target_lengths(x, y, z, pitch, roll):
    """Wrapper to get just the string lengths from your math."""
    local_coordinates = calculate_local_coordinates(pitch, roll)
    E_global_coords = np.array([x, y, z]) + local_coordinates
    target_lengths = calculate_string_lengths(E_global_coords)
    return target_lengths


# --- MuJoCo Setup ---
model = mujoco.MjModel.from_xml_path("cable_robot.xml")
data = mujoco.MjData(model)

# PD Controller Gains
Kp = 500.0  # Spring stiffness
Kv = 20.0  # Damping


# --- The Magic Callback ---
# MuJoCo will automatically call this function every single physics step.
def my_controller(model, data):
    # Use MuJoCo's internal simulation time instead of the real-world clock!
    # This keeps the physics perfectly synced even if the viewer lags.
    t = data.time

    # --- 5DOF Target Trajectory ---
    target_x = 0.3 * m.sin(t * 0.5)
    target_y = 0.3 * m.cos(t * 0.5)
    target_z = 1.0 + 0.2 * m.sin(t)
    target_pitch = m.radians(10 * m.sin(t * 0.8))
    target_roll = m.radians(10 * m.cos(t * 0.8))

    # Get the desired cable lengths for this position
    target_L = calculate_target_lengths(
        target_x, target_y, target_z, target_pitch, target_roll
    )

    # --- Custom PD Controller (Pull-Only) ---
    for i in range(4):
        current_length = data.ten_length[i]
        current_velocity = data.ten_velocity[i]

        error = current_length - target_L[i]
        tension = (Kp * error) - (Kv * current_velocity)

        # Physics Constraint: Cables cannot push!
        if tension < 0:
            tension = 0

        data.ctrl[i] = tension


# 1. Register our custom Python function with the MuJoCo engine
mujoco.set_mjcb_control(my_controller)

print("Starting simulation... (Press ESC to close the viewer)")

# 2. Launch the standard, blocking viewer.
# You can now run this using standard `python mujoco_runner.py`!
mujoco.viewer.launch(model, data)

# Optional: Clear the callback when the viewer closes so it doesn't persist in memory
mujoco.set_mjcb_control(None)
