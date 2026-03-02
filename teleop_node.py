#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler
import sys
import termios
import tty

msg = """
Control Your Cable Robot!
---------------------------
Moving around:      Tilting:        Speed:
    Q   W   E           U           T : Faster (+)
    A   S   D       H   J   K       G : Slower (-)

W/S : +/- X      U/J : +/- Pitch
A/D : +/- Y      H/K : +/- Roll
Q/E : +/- Z      R   : Reset Origin

CTRL-C to quit
"""

# The base values are multiplied by the speed_multiplier
move_bindings = {
    "w": (1.0, 0, 0, 0, 0),
    "s": (-1.0, 0, 0, 0, 0),
    "a": (0, 1.0, 0, 0, 0),
    "d": (0, -1.0, 0, 0, 0),
    "q": (0, 0, 1.0, 0, 0),
    "e": (0, 0, -1.0, 0, 0),
    "u": (0, 0, 0, 1.0, 0),
    "j": (0, 0, 0, -1.0, 0),
    "h": (0, 0, 0, 0, 1.0),
    "k": (0, 0, 0, 0, -1.0),
}


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_node")
        self.publisher = self.create_publisher(Pose, "/target_pose", 10)

        # Initial State
        self.x, self.y, self.z = 0.0, 0.0, -1.0
        self.pitch, self.roll = 0.0, 0.0

        # Incremental Step Sizes
        self.pos_step = 0.01  # 1cm base step
        self.rot_step = 0.05  # ~3 degrees base step
        self.speed_multiplier = 5.0

        self.timer = self.create_timer(0.1, self.publish_pose)
        self.get_logger().info("Teleop Node Started")

    def publish_pose(self):
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = self.z

        q = quaternion_from_euler(self.roll, self.pitch, 0.0)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        self.publisher.publish(pose)


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = TeleopNode()

    print(msg)

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key in move_bindings.keys():
                dx, dy, dz, dp, dr = move_bindings[key]
                # Apply multipliers
                node.x += dx * node.pos_step * node.speed_multiplier
                node.y += dy * node.pos_step * node.speed_multiplier
                node.z += dz * node.pos_step * node.speed_multiplier
                node.pitch += dp * node.rot_step * node.speed_multiplier
                node.roll += dr * node.rot_step * node.speed_multiplier

            elif key == "t":
                node.speed_multiplier += 0.5
                print(f"Speed increased to: {node.speed_multiplier:.1f}")

            elif key == "g":
                node.speed_multiplier = max(0.1, node.speed_multiplier - 0.1)
                print(f"Speed decreased to: {node.speed_multiplier:.1f}")

            elif key == "r":
                node.x, node.y, node.z = 0.0, 0.0, 1.0
                node.pitch, node.roll = 0.0, 0.0
                print("Reset to origin")

            elif key == "\x03":  # Ctrl+C
                break

            # Print status line (clears line and prints new values)
            sys.stdout.write(
                f"\rPos: ({node.x:.2f}, {node.y:.2f}, {node.z:.2f}) | Speed: {node.speed_multiplier:.1f}   "
            )
            sys.stdout.flush()

            rclpy.spin_once(node, timeout_sec=0)

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
