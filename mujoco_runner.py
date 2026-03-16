import mujoco
import mujoco.viewer
import time
import math as m
import numpy as np

# Import your exact kinematics logic
# Adjust the import path if necessary based on your folder structure
from kinematics import (
    calculate_string_lengths,
    calculate_local_coordinates,
)


def calculate_target_lengths(x, y, z, pitch, roll):
    """Wrapper for your kinematics to return exactly the target string lengths."""
    local_coordinates = calculate_local_coordinates(pitch, roll)
    E_global_coords = np.array([x, y, z]) + local_coordinates
    return calculate_string_lengths(E_global_coords)


# Load the XML file we just created
model = mujoco.MjModel.from_xml_path("robot.xml")
data = mujoco.MjData(model)

# PD Controller Gains
# Kp acts like the spring stiffness of the motor trying to hit the target length
# Kv acts like the motor's braking friction to stop oscillations
Kp = 500.0
Kv = 20.0

# We want the anchors to be Z=2.0 in the math to match the XML
# Note: You may need to update your A_frame_coods in your kinematics file so the Z values are 2.0 instead of 0.
Z_OFFSET = 2.0

with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()

    while viewer.is_running():
        step_start = time.time()
        t = time.time() - start_time

        # --- 1. Generate 5DOF Target Trajectory ---
        # Slowly oscillate x, y, z, pitch, and roll over time
        target_x = 0.3 * m.sin(t * 0.5)  # Move left/right 30cm
        target_y = 0.3 * m.cos(t * 0.5)  # Move forward/backward 30cm
        target_z = 1.0 + 0.2 * m.sin(t)  # Bob up and down around Z=1.0m
        target_pitch = m.radians(10 * m.sin(t * 0.8))  # Tilt pitch +/- 10 deg
        target_roll = m.radians(10 * m.cos(t * 0.8))  # Tilt roll +/- 10 deg

        # --- 2. Inverse Kinematics ---
        # Get the desired cable lengths for this position
        target_L = calculate_target_lengths(
            target_x, target_y, target_z, target_pitch, target_roll
        )

        # --- 3. Custom PD Controller (Pull-Only) ---
        for i in range(4):
            # Read current physics state from MuJoCo
            current_length = data.ten_length[i]
            current_velocity = data.ten_velocity[i]

            # Calculate required tension
            # If current length > target length, error is positive, we need to PULL
            error = current_length - target_L[i]
            tension = (Kp * error) - (Kv * current_velocity)

            # CRITICAL PHYSICS FIX: Cables cannot push!
            if tension < 0:
                tension = 0

            # Apply tension to the MuJoCo motor
            data.ctrl[i] = tension

        # Step the physics engine
        mujoco.mj_step(model, data)

        # Update the viewer
        viewer.sync()

        # Keep the simulation running roughly in real-time
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
