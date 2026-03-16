import mujoco
import mujoco.viewer
import time
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


# --- MuJoCo Setup & Control Loop ---
model = mujoco.MjModel.from_xml_path("cable_robot.xml")
data = mujoco.MjData(model)

# PD Controller Gains
Kp = 500.0  # Spring stiffness (How hard to pull to reach target length)
Kv = 20.0  # Damping (Braking friction to stop oscillations)

with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()

    while viewer.is_running():
        step_start = time.time()
        t = time.time() - start_time

        # --- 5DOF Target Trajectory ---
        target_x = 0.3 * m.sin(t * 0.5)  # Left/right 30cm
        target_y = 0.3 * m.cos(t * 0.5)  # Forward/backward 30cm
        target_z = 1.0 + 0.2 * m.sin(t)  # Bob up and down around Z=1.0m
        target_pitch = m.radians(10 * m.sin(t * 0.8))  # Tilt pitch
        target_roll = m.radians(10 * m.cos(t * 0.8))  # Tilt roll

        # Get the desired cable lengths for this position
        target_L = calculate_target_lengths(
            target_x, target_y, target_z, target_pitch, target_roll
        )

        # --- Custom PD Controller (Pull-Only) ---
        for i in range(4):
            current_length = data.ten_length[i]
            current_velocity = data.ten_velocity[i]

            # If current length > target length, error is positive -> PULL
            error = current_length - target_L[i]
            tension = (Kp * error) - (Kv * current_velocity)

            # Physics Constraint: Cables cannot push!
            if tension < 0:
                tension = 0

            data.ctrl[i] = tension

        mujoco.mj_step(model, data)
        viewer.sync()

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
