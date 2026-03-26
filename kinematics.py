import math as m
import numpy as np

# from tf_transformations import quaternion_from_euler
import logging

logging.basicConfig(level=logging.INFO)

END_EFFECTOR_MASS = 0.100  # kg
S = 2.0  # m - Outside frame size
E_W = 0.2  # m - End effector width
E_L = 0.2  # m - End effector length
E_H = 0.01  # m - End effector height
SPOOL_RADIUS = 0.02  # m - Spool radius
Z = 2.0  # m - Height of the frame anchors (MuJoCo's Z=0 is the ground, so we place anchors at Z=2.0)
Z_EFF = 1.0  # m - Default height of the end effector (for initial kinematics calculation)

A_frame_coods = np.array(
    [
        np.array([S / 2, S / 2, Z]),
        np.array([-S / 2, S / 2, Z]),
        np.array([-S / 2, -S / 2, Z]),
        np.array([S / 2, -S / 2, Z]),
    ]
)

E_local = np.array(
    [
        np.array([E_W / 2, E_L / 2, Z_EFF]),
        np.array([-E_W / 2, E_L / 2, Z_EFF]),
        np.array([-E_W / 2, -E_L / 2, Z_EFF]),
        np.array([E_W / 2, -E_L / 2, Z_EFF]),
    ]
)

# def get_quaternion(pitch, roll):
#     # This function expects roll (x), pitch (y), and yaw (z).
#     # It returns the quaternion as a list: [x, y, z, w]
#     return quaternion_from_euler(roll, pitch, 0.0)


def calculate_rotation_matrix(pitch, roll):
    # Using ZYX Euler angles (yaw, pitch, roll) for rotation
    return np.array(  # Rotation around X-axis (roll)
        [
            [1, 0, 0],
            [0, m.cos(roll), -m.sin(roll)],
            [0, m.sin(roll), m.cos(roll)],
        ]
    ) @ np.array(  # Rotation around Y-axis (pitch)
        [
            [m.cos(pitch), 0, m.sin(pitch)],
            [0, 1, 0],
            [-m.sin(pitch), 0, m.cos(pitch)],
        ]
    )


def calculate_local_coordinates(pitch, roll):
    rotation_matrix = calculate_rotation_matrix(pitch, roll)
    return np.dot(rotation_matrix, E_local.T).T


def calculate_string_lengths(E_global_coords):
    return np.linalg.norm(A_frame_coods - E_global_coords, axis=1)


def calculate_kinematics(x, y, z, pitch, roll):
    local_coordinates = calculate_local_coordinates(pitch, roll)
    E_global_coords = np.array([x, y, z]) + local_coordinates
    logging.info("Local Coordinates:\n%s", local_coordinates)
    logging.info("End Effector Global Coordinates:\n%s", E_global_coords)
    L_strings = calculate_string_lengths(E_global_coords)
    logging.info("String Lengths:\n%s", L_strings)
    angles = (
        L_strings * 360 / (2 * m.pi * SPOOL_RADIUS)
    )  # Convert length to angle (degrees)
    logging.info("Motor Angles (degrees):\n%s", angles)
    return E_global_coords


if __name__ == "__main__":
    x = 0  # m
    y = 0  # m
    z = 0.0  # m
    pitch = m.radians(10)  # radians
    roll = m.radians(0)  # radians

    calculate_kinematics(x, y, z, pitch, roll)
