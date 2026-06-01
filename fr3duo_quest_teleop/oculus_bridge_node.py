#!/usr/bin/env python3
import os
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
import numpy as np


def add_vendored_oculus_reader_to_path():
    workspace_src_path = os.path.join(
        '/home',
        'amal',
        'franka_ros2_ws',
        'src',
        'fr3duo_quest_teleop',
        'oculus_reader',
    )
    if os.path.isdir(workspace_src_path) and workspace_src_path not in sys.path:
        sys.path.insert(0, workspace_src_path)


add_vendored_oculus_reader_to_path()
from oculus_reader.reader import OculusReader


class OculusBridge(Node):
    def __init__(self):
        super().__init__('oculus_bridge_node')
        
        # Initialize the Oculus Reader (must be connected via ADB)
        try:
            self.oculus = OculusReader()
            self.get_logger().info("Oculus Quest connected successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Oculus: {e}")
            raise e

        # Publishers
        self.left_pose_pub = self.create_publisher(PoseStamped, '/oculus/left_controller_pose', 10)
        self.right_pose_pub = self.create_publisher(PoseStamped, '/oculus/right_controller_pose', 10)
        self.left_btn_pub = self.create_publisher(Joy, '/oculus/left_controller_buttons', 10)
        self.right_btn_pub = self.create_publisher(Joy, '/oculus/right_controller_buttons', 10)

        # Polling Timer (e.g., 50Hz)
        self.timer = self.create_timer(0.02, self.publish_data)

    def publish_data(self):
        transformations, buttons = self.oculus.get_transformations_and_buttons()
        if transformations is None:
            transformations = {}
        if buttons is None:
            buttons = {}

        # Handle Left Controller
        if 'l' in transformations:
            self.publish_pose(transformations['l'], self.left_pose_pub, 'left_controller')
        if self.has_side_button_data(buttons, 'l'):
            self.publish_buttons(buttons, 'l', self.left_btn_pub)

        # Handle Right Controller
        if 'r' in transformations:
            self.publish_pose(transformations['r'], self.right_pose_pub, 'right_controller')
        if self.has_side_button_data(buttons, 'r'):
            self.publish_buttons(buttons, 'r', self.right_btn_pub)

    def publish_pose(self, matrix, publisher, frame_id):
        # Extract translation
        pos = matrix[:3, 3]
        
        # Extract rotation matrix to quaternion (using scipy or basic math)
        from scipy.spatial.transform import Rotation as R
        rot = R.from_matrix(matrix[:3, :3]).as_quat()

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pos[0], pos[1], pos[2]
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = rot[0], rot[1], rot[2], rot[3]
        publisher.publish(msg)

    def has_side_button_data(self, buttons_dict, side):
        if side == 'l':
            side_keys = ('X', 'Y', 'LThU', 'LJ', 'LG', 'LTr', 'leftJS', 'leftGrip', 'leftTrig')
        else:
            side_keys = ('A', 'B', 'RThU', 'RJ', 'RG', 'RTr', 'rightJS', 'rightGrip', 'rightTrig')
        return any(key in buttons_dict for key in side_keys)

    def button_value(self, buttons_dict, digital_key, analog_key=None, threshold=0.5):
        if bool(buttons_dict.get(digital_key, False)):
            return 1
        if analog_key is None:
            return 0

        value = buttons_dict.get(analog_key, 0.0)
        if isinstance(value, (list, tuple)):
            value = value[0] if value else 0.0
        try:
            return int(float(value) > threshold)
        except (TypeError, ValueError):
            return 0

    def analog_value(self, buttons_dict, analog_key, digital_key=None):
        value = buttons_dict.get(analog_key, 0.0)
        if isinstance(value, (list, tuple)):
            value = value[0] if value else 0.0
        try:
            analog = float(value)
        except (TypeError, ValueError):
            analog = 0.0
        if digital_key is not None and bool(buttons_dict.get(digital_key, False)):
            analog = max(analog, 1.0)
        return min(max(analog, 0.0), 1.0)

    def axis_value(self, buttons_dict, key, index, default=0.0):
        value = buttons_dict.get(key, ())
        if not isinstance(value, (list, tuple)) or len(value) <= index:
            return default
        try:
            return float(value[index])
        except (TypeError, ValueError):
            return default

    def publish_buttons(self, buttons_dict, side, publisher):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()

        if side == 'l':
            primary_key = 'X'
            secondary_key = 'Y'
            grip_key = 'LG'
            trigger_key = 'LTr'
            grip_axis_key = 'leftGrip'
            trigger_axis_key = 'leftTrig'
            joystick_key = 'leftJS'
        else:
            primary_key = 'A'
            secondary_key = 'B'
            grip_key = 'RG'
            trigger_key = 'RTr'
            grip_axis_key = 'rightGrip'
            trigger_axis_key = 'rightTrig'
            joystick_key = 'rightJS'

        # Button mapping consumed by franka_teleop_node:
        #   0: A/X, 1: grip deadman, 2: trigger gripper command, 3: B/Y
        msg.buttons = [
            self.button_value(buttons_dict, primary_key),
            self.button_value(buttons_dict, grip_key, grip_axis_key),
            self.button_value(buttons_dict, trigger_key, trigger_axis_key),
            self.button_value(buttons_dict, secondary_key),
        ]
        msg.axes = [
            self.axis_value(buttons_dict, joystick_key, 0),
            self.axis_value(buttons_dict, joystick_key, 1),
            self.analog_value(buttons_dict, grip_axis_key, grip_key),
            self.analog_value(buttons_dict, trigger_axis_key, trigger_key),
        ]
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OculusBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
