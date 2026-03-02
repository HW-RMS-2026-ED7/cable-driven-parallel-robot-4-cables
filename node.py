#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Pose, TransformStamped, Point
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster
import logging

from cable_robot_kinematics.kinematics import calculate_kinematics, E_W, E_L, E_H, A_frame_coods

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class InverseKinematicsNode(Node):
    def __init__(self):
        """Initialize the ROS2 node and set up the subscriber."""
        super().__init__("inverse_kinematics_node")

        # Define QoS profile for reliability
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Create a Subscriber
        # - Topic: /target_pose
        # - Message type: Pose
        # - Callback: self.pose_callback
        self.subscriber = self.create_subscription(
            Pose, "/target_pose", self.pose_callback, qos_profile
        )

        # Initialize TransformBroadcaster to publish frame relationships
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize Publisher to send Marker messages for visualization in RViz
        self.marker_publisher = self.create_publisher(
            Marker, "/visualization_marker", qos_profile
        )

        self.get_logger().info(
            "Inverse Kinematics Node initialized. Listening on /target_pose"
        )

    def pose_callback(self, pose_msg):
        """
        Callback function that executes every time a new Pose message arrives.

        Unpacks the pose data and calculates the inverse kinematics.

        Args:
            pose_msg (geometry_msgs.msg.Pose): The incoming pose message containing
                                               position (x, y, z) and orientation (quaternion)
        """
        try:
            # Extract position (x, y, z)
            x = pose_msg.position.x
            y = pose_msg.position.y
            z = pose_msg.position.z

            # Extract quaternion (x, y, z, w)
            quaternion = (
                pose_msg.orientation.x,
                pose_msg.orientation.y,
                pose_msg.orientation.z,
                pose_msg.orientation.w,
            )

            # Convert quaternion to Euler angles (roll, pitch, yaw)
            roll, pitch, yaw = euler_from_quaternion(quaternion)

            self.get_logger().info(
                f"Received pose: position=({x:.3f}, {y:.3f}, {z:.3f}), "
                f"euler=(roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f})"
            )

            # Calculate inverse kinematics and get platform corner coordinates
            E_global_coords = calculate_kinematics(x, y, z, pitch, roll)

            # Broadcast the transform from base_link to end_effector
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "base_link"
            transform.child_frame_id = "end_effector"

            # Set translation (position)
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z

            # Convert Euler angles back to quaternion for the transform
            quat = quaternion_from_euler(roll, pitch, yaw)
            transform.transform.rotation.x = quat[0]
            transform.transform.rotation.y = quat[1]
            transform.transform.rotation.z = quat[2]
            transform.transform.rotation.w = quat[3]

            # Publish the transform
            self.tf_broadcaster.sendTransform(transform)
            self.get_logger().debug("Published transform: base_link -> end_effector")

            # Create and publish the platform marker
            platform_marker = Marker()
            platform_marker.header.frame_id = "end_effector"
            platform_marker.header.stamp = self.get_clock().now().to_msg()

            platform_marker.ns = "platform"
            platform_marker.id = 0

            # Set the shape type and action
            platform_marker.type = Marker.CUBE
            platform_marker.action = Marker.ADD

            # Set the scale based on platform dimensions
            platform_marker.scale.x = E_W
            platform_marker.scale.y = E_L
            platform_marker.scale.z = E_H

            # Set the color (blue with full opacity)
            platform_marker.color.r = 0.0
            platform_marker.color.g = 0.0
            platform_marker.color.b = 1.0
            platform_marker.color.a = 1.0

            # Publish the marker
            self.marker_publisher.publish(platform_marker)
            self.get_logger().debug("Published platform marker")

            # Create and publish the strings marker (LINE_LIST connecting anchors to platform)
            strings_marker = Marker()
            strings_marker.header.frame_id = "base_link"
            strings_marker.header.stamp = self.get_clock().now().to_msg()
            strings_marker.ns = "strings"
            strings_marker.id = 1

            # Set the shape type and action
            strings_marker.type = Marker.LINE_LIST
            strings_marker.action = Marker.ADD

            # Set scale (line width)
            strings_marker.scale.x = 0.01  # 1cm line width

            # Set the color (white for cables)
            strings_marker.color.r = 1.0
            strings_marker.color.g = 1.0
            strings_marker.color.b = 1.0
            strings_marker.color.a = 1.0

            # Build the list of points: [Anchor 1, Corner 1, Anchor 2, Corner 2, ...]
            strings_marker.points = []
            for i in range(4):
                # Add anchor point
                anchor_point = Point()
                anchor_point.x = A_frame_coods[i][0]
                anchor_point.y = A_frame_coods[i][1]
                anchor_point.z = A_frame_coods[i][2]
                strings_marker.points.append(anchor_point)

                # Add platform corner point
                corner_point = Point()
                corner_point.x = E_global_coords[i][0]
                corner_point.y = E_global_coords[i][1]
                corner_point.z = E_global_coords[i][2]
                strings_marker.points.append(corner_point)

            # Publish the strings marker
            self.marker_publisher.publish(strings_marker)
            self.get_logger().debug("Published strings marker")

        except Exception as e:
            self.get_logger().error(f"Error in pose_callback: {e}", exc_info=True)

    def spin(self):
        """Keep the node running and listening for messages."""
        self.get_logger().info("Node is running. Press Ctrl+C to stop.")

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()
