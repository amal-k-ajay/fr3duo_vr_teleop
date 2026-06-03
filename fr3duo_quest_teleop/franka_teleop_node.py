#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import Joy
from control_msgs.action import GripperCommand
from franka_msgs.action import Homing
from std_srvs.srv import Trigger

from rclpy.action import ActionClient
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class FrankaQuestTeleop(Node):
    def __init__(self):
        super().__init__('franka_teleop_node')

        # --- Parameters ---
        self.declare_parameter('hand_side', 'right')
        self.declare_parameter('linear_multiplier', 1.5)
        self.declare_parameter('angular_multiplier', 1.0)
        self.declare_parameter('kp_linear', 4.0)
        self.declare_parameter('kp_angular', 3.0)
        self.declare_parameter('collision_distance', 0.25) # Meters
        self.declare_parameter('base_frame', '')
        self.declare_parameter('ee_frame', '')
        self.declare_parameter('other_ee_frame', '')
        self.declare_parameter('pose_topic', '')
        self.declare_parameter('button_topic', '')
        self.declare_parameter('twist_topic', '')
        self.declare_parameter('gripper_action', '')
        self.declare_parameter('gripper_homing_action', '')
        self.declare_parameter('gripper_open_width', 0.080)
        self.declare_parameter('gripper_closed_width', 0.0)
        self.declare_parameter('gripper_max_command_width', 0.080)
        self.declare_parameter('gripper_max_effort', 20.0)
        self.declare_parameter('gripper_trigger_axis_index', 3)
        self.declare_parameter('gripper_trigger_threshold', 0.5)
        self.declare_parameter('gripper_open_on_start', True)
        self.declare_parameter('gripper_homing_on_start', False)
        self.declare_parameter('servo_start_service', '')
        self.declare_parameter(
            'controller_to_robot_rotation',
            [
                0.0, 0.0, -1.0,
                -1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
            ],
        )

        self.side = self.get_parameter('hand_side').value
        self.lin_mult = self.get_parameter('linear_multiplier').value
        self.ang_mult = self.get_parameter('angular_multiplier').value
        self.kp_lin = self.get_parameter('kp_linear').value
        self.kp_ang = self.get_parameter('kp_angular').value
        self.collision_dist = self.get_parameter('collision_distance').value
        self.gripper_open_width = self.get_parameter('gripper_open_width').value
        self.gripper_closed_width = self.get_parameter('gripper_closed_width').value
        self.gripper_max_command_width = self.get_parameter('gripper_max_command_width').value
        self.gripper_max_effort = self.get_parameter('gripper_max_effort').value
        self.gripper_trigger_axis_index = self.get_parameter('gripper_trigger_axis_index').value
        self.gripper_trigger_threshold = self.get_parameter('gripper_trigger_threshold').value
        self.gripper_open_on_start = self.get_parameter('gripper_open_on_start').value
        self.gripper_homing_on_start = self.get_parameter('gripper_homing_on_start').value
        self.controller_to_robot_rotation = self.load_controller_to_robot_rotation()

        side_prefix = 'left_fr3v2' if self.side == 'left' else 'right_fr3v2'
        other_prefix = 'right_fr3v2' if self.side == 'left' else 'left_fr3v2'
        self.base_frame = self._param_or_default('base_frame', f'{side_prefix}_link0')
        self.ee_frame = self._param_or_default('ee_frame', f'{side_prefix}_hand_tcp')
        self.other_ee_frame = self._param_or_default('other_ee_frame', f'{other_prefix}_hand_tcp')
        self.pose_topic = self._param_or_default('pose_topic', f'/oculus/{self.side}_controller_pose')
        self.button_topic = self._param_or_default('button_topic', f'/oculus/{self.side}_controller_buttons')
        self.twist_topic = self._param_or_default('twist_topic', f'/{self.side}/servo_node/delta_twist_cmds')
        self.gripper_action_name = self._param_or_default(
            'gripper_action',
            f'/{self.side}_hand_controller/gripper_cmd',
        )
        self.gripper_homing_action_name = self._param_or_default(
            'gripper_homing_action',
            '',
        )
        self.servo_start_service = self._param_or_default(
            'servo_start_service',
            f'/{self.side}/servo_node/start_servo',
        )

        # --- TF2 Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Publishers & Subscribers ---
        self.twist_pub = self.create_publisher(TwistStamped, self.twist_topic, 10)
        
        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self.pose_cb, 10)
        self.button_sub = self.create_subscription(Joy, self.button_topic, self.button_cb, 10)
        self.gripper_client = ActionClient(self, GripperCommand, self.gripper_action_name)
        self.gripper_homing_client = None
        if self.gripper_homing_action_name:
            self.gripper_homing_client = ActionClient(
                self,
                Homing,
                self.gripper_homing_action_name,
            )
        self.servo_start_client = self.create_client(Trigger, self.servo_start_service)

        # --- State Tracking ---
        self.grip_pressed = False
        self.prev_grip_pressed = False
        self.last_logged_grip_pressed = False
        self.trigger_pressed = False
        self.prev_trigger_pressed = False
        self.gripper_is_open = True

        # Anchors for relative tracking
        self.c0_pos = np.zeros(3)
        self.c0_rot = R.identity()
        self.r0_pos = np.zeros(3)
        self.r0_rot = R.identity()
        self.servo_started = False
        self.servo_start_future = None
        self.servo_start_timer = self.create_timer(1.0, self.start_servo_if_needed)
        self.gripper_start_opened = False
        self.gripper_start_open_timer = None
        self.gripper_homing_goal_future = None
        self.gripper_homing_result_future = None
        if self.gripper_open_on_start or self.gripper_homing_on_start:
            self.gripper_start_open_timer = self.create_timer(1.0, self.open_gripper_on_start)

        self.get_logger().info(
            f"{self.side.capitalize()} teleop: {self.pose_topic} -> {self.twist_topic}, "
            f"frames {self.base_frame} -> {self.ee_frame}"
        )

    def _param_or_default(self, name, default):
        value = self.get_parameter(name).value
        return value if value else default

    def load_controller_to_robot_rotation(self):
        values = self.get_parameter('controller_to_robot_rotation').value
        matrix = np.array(values, dtype=float)
        if matrix.size != 9:
            raise ValueError(
                'controller_to_robot_rotation must contain exactly 9 values.'
            )
        matrix = matrix.reshape((3, 3))
        if not np.allclose(matrix @ matrix.T, np.eye(3), atol=1e-3):
            self.get_logger().warn(
                'controller_to_robot_rotation is not orthonormal; movement '
                'may scale or skew unexpectedly.'
            )
        return matrix

    def start_servo_if_needed(self):
        if self.servo_started:
            self.servo_start_timer.cancel()
            return

        if self.servo_start_future is not None:
            if not self.servo_start_future.done():
                return
            result = self.servo_start_future.result()
            if result is not None and result.success:
                self.servo_started = True
                self.get_logger().info(f'Started MoveIt Servo via {self.servo_start_service}.')
                self.servo_start_timer.cancel()
                return
            message = result.message if result is not None else 'service call failed'
            self.get_logger().warn(
                f'MoveIt Servo start request did not succeed: {message}',
                throttle_duration_sec=2.0,
            )
            self.servo_start_future = None

        if not self.servo_start_client.service_is_ready():
            self.get_logger().warn(
                f'Waiting for MoveIt Servo start service {self.servo_start_service}.',
                throttle_duration_sec=3.0,
            )
            return

        self.servo_start_future = self.servo_start_client.call_async(Trigger.Request())

    def open_gripper_on_start(self):
        if self.gripper_start_opened:
            if self.gripper_start_open_timer is not None:
                self.gripper_start_open_timer.cancel()
            return

        if self.gripper_homing_on_start and self.gripper_homing_client is not None:
            if self.home_gripper_on_start():
                self.gripper_start_opened = True
                self.gripper_is_open = True
                if self.gripper_start_open_timer is not None:
                    self.gripper_start_open_timer.cancel()
            return

        if not self.gripper_client.server_is_ready():
            self.get_logger().warn(
                f"Waiting for gripper action {self.gripper_action_name} before startup open.",
                throttle_duration_sec=3.0,
            )
            return

        self.gripper_is_open = True
        self.command_gripper(self.gripper_open_width)
        self.gripper_start_opened = True
        self.get_logger().info(
            f'{self.side.capitalize()} gripper initialized open '
            f'to {float(self.gripper_open_width):.6f} m.'
        )
        if self.gripper_start_open_timer is not None:
            self.gripper_start_open_timer.cancel()

    def home_gripper_on_start(self):
        if self.gripper_homing_result_future is not None:
            if not self.gripper_homing_result_future.done():
                return False
            result = self.gripper_homing_result_future.result().result
            if result.success:
                self.get_logger().info(
                    f'{self.side.capitalize()} gripper homed and initialized fully open.'
                )
                return True
            self.get_logger().warn(
                f'{self.side.capitalize()} gripper homing failed: {result.error}. '
                'Falling back to configured open width.'
            )
            self.gripper_homing_on_start = False
            return False

        if self.gripper_homing_goal_future is not None:
            if not self.gripper_homing_goal_future.done():
                return False
            goal_handle = self.gripper_homing_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().warn(
                    f'{self.side.capitalize()} gripper homing goal was rejected. '
                    'Falling back to configured open width.'
                )
                self.gripper_homing_on_start = False
                return False
            self.gripper_homing_result_future = goal_handle.get_result_async()
            return False

        if not self.gripper_homing_client.server_is_ready():
            self.get_logger().warn(
                f"Waiting for gripper homing action {self.gripper_homing_action_name}.",
                throttle_duration_sec=3.0,
            )
            return False

        self.get_logger().info(
            f'{self.side.capitalize()} gripper homing on startup.'
        )
        self.gripper_homing_goal_future = self.gripper_homing_client.send_goal_async(
            Homing.Goal()
        )
        return False

    def controller_delta_to_robot_delta(self, controller_delta):
        return self.controller_to_robot_rotation @ controller_delta

    def controller_rotation_to_robot_rotation(self, controller_rotation):
        controller_rot_vec = controller_rotation.as_rotvec()
        robot_rot_vec = self.controller_to_robot_rotation @ controller_rot_vec
        return R.from_rotvec(robot_rot_vec)

    def get_robot_pose(self):
        """Looks up the current end-effector pose via TF."""
        try:
            trans = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, rclpy.time.Time())
            pos = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
            quat = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
            rot = R.from_quat(quat)
            return pos, rot
        except (
            LookupException,
            ConnectivityException,
            ExtrapolationException,
        ) as e:
            self.get_logger().debug(f"TF Lookup failed: {e}")
            return None, None

    def check_collision(self):
        """Checks the distance between left and right end-effectors."""
        try:
            # Look up transform from this arm's hand to the other arm's hand
            trans = self.tf_buffer.lookup_transform(self.ee_frame, self.other_ee_frame, rclpy.time.Time())
            distance = np.linalg.norm([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
            return distance < self.collision_dist
        except (LookupException, ConnectivityException, ExtrapolationException):
            # If TF fails (e.g. other robot offline), assume safe to move locally
            return False

    def button_cb(self, msg):
        # Adjust indices according to oculus_reader outputs
        # Usually: index 1 is Grip, index 2 is Trigger
        if len(msg.buttons) < 3:
            self.get_logger().warn('Ignoring Joy message with fewer than 3 buttons.', throttle_duration_sec=2.0)
            return
        self.grip_pressed = bool(msg.buttons[1])
        trigger_value = self.trigger_value_from_joy(msg)
        self.trigger_pressed = trigger_value > float(self.gripper_trigger_threshold)

        if self.grip_pressed != self.last_logged_grip_pressed:
            state = 'pressed' if self.grip_pressed else 'released'
            self.get_logger().info(f'{self.side.capitalize()} grip deadman {state}.')
            self.last_logged_grip_pressed = self.grip_pressed

        if self.trigger_pressed and not self.prev_trigger_pressed:
            self.toggle_gripper()
        self.prev_trigger_pressed = self.trigger_pressed

    def trigger_value_from_joy(self, msg):
        try:
            axis_index = int(self.gripper_trigger_axis_index)
        except (TypeError, ValueError):
            axis_index = 3

        if len(msg.axes) > axis_index:
            try:
                return min(max(float(msg.axes[axis_index]), 0.0), 1.0)
            except (TypeError, ValueError):
                pass

        return 1.0 if len(msg.buttons) > 2 and bool(msg.buttons[2]) else 0.0

    def toggle_gripper(self):
        self.gripper_is_open = not self.gripper_is_open
        target_width = (
            self.gripper_open_width if self.gripper_is_open else self.gripper_closed_width
        )
        state = 'open' if self.gripper_is_open else 'closed'
        self.get_logger().info(f'{self.side.capitalize()} gripper toggled {state}.')
        self.command_gripper(target_width)

    def clamp_gripper_width(self, width):
        max_width = float(self.gripper_max_command_width)
        closed_width = float(self.gripper_closed_width)
        min_width = min(closed_width, max_width)
        max_width = max(closed_width, max_width)
        clamped_width = min(max(float(width), min_width), max_width)
        if clamped_width != float(width):
            self.get_logger().warn(
                f'Clamped gripper command from {float(width):.6f} to {clamped_width:.6f}.',
                throttle_duration_sec=2.0,
            )
        return clamped_width

    def command_gripper(self, width):
        if not self.gripper_client.server_is_ready():
            self.get_logger().warn(
                f"Gripper action {self.gripper_action_name} is not ready.",
                throttle_duration_sec=2.0,
            )
            return

        goal = GripperCommand.Goal()
        goal.command.position = 0.5 * self.clamp_gripper_width(width)
        goal.command.max_effort = float(self.gripper_max_effort)
        self.gripper_client.send_goal_async(goal)

    def pose_cb(self, msg):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.base_frame

        # 1. Collision Safety Override
        if self.check_collision():
            self.get_logger().warn(f"COLLISION WARNING! Dual arms are too close. Halting {self.side} arm.", throttle_duration_sec=1.0)
            self.publish_zero_twist(twist_msg)
            return

        # 2. Get Current Controller Pose
        c_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        c_quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        c_rot = R.from_quat(c_quat)

        # 3. Get Current Robot Pose
        r_pos, r_rot = self.get_robot_pose()
        if r_pos is None:
            return # Skip cycle if TF isn't ready

        # 4. Teleoperation Logic
        if self.grip_pressed:
            # Rising Edge: Lock in reference frames
            if not self.prev_grip_pressed:
                self.c0_pos = c_pos
                self.c0_rot = c_rot
                self.r0_pos = r_pos
                self.r0_rot = r_rot
                self.prev_grip_pressed = True
                return

            # --- POSITION TRACKING ---
            # Calculate controller delta and apply multiplier
            delta_pos = self.controller_delta_to_robot_delta(c_pos - self.c0_pos) * self.lin_mult
            target_pos = self.r0_pos + delta_pos
            
            # Position Error (P-Control)
            pos_error = target_pos - r_pos
            twist_msg.twist.linear.x = self.kp_lin * pos_error[0]
            twist_msg.twist.linear.y = self.kp_lin * pos_error[1]
            twist_msg.twist.linear.z = self.kp_lin * pos_error[2]

            # --- ORIENTATION TRACKING ---
            # Calculate rotational delta of controller
            delta_rot = c_rot * self.c0_rot.inv()
            
            # Target robot orientation
            target_rot = self.controller_rotation_to_robot_rotation(delta_rot) * self.r0_rot
            
            # Calculate orientation error (Target * Current_Inv)
            error_rot = target_rot * r_rot.inv()
            
            # Convert error to axis-angle (which translates directly to angular velocity vector)
            rot_vec = error_rot.as_rotvec()
            twist_msg.twist.angular.x = self.kp_ang * rot_vec[0] * self.ang_mult
            twist_msg.twist.angular.y = self.kp_ang * rot_vec[1] * self.ang_mult
            twist_msg.twist.angular.z = self.kp_ang * rot_vec[2] * self.ang_mult

            self.twist_pub.publish(twist_msg)

        else:
            # Send explicit zero twist commands when clutch is released
            if self.prev_grip_pressed:
                self.prev_grip_pressed = False
            
            self.publish_zero_twist(twist_msg)

    def publish_zero_twist(self, twist_msg):
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0
        self.twist_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrankaQuestTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
