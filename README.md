# FR3 Duo Quest Teleop

ROS 2 package for controlling a dual Franka FR3 setup from Meta Quest controllers.

The package supports two launch paths:

- Simulation/RViz using `fr3_duo_moveit_config`
- Hardware bringup using two real FR3 robots through `franka_bringup`

The Quest grip button is used as the deadman switch. While grip is held, controller translation is mapped to end-effector Cartesian motion. The trigger button commands the gripper.

## Build

From the workspace root:

```bash
cd /home/amal/franka_ros2_ws
colcon build --packages-select fr3duo_quest_teleop --symlink-install
source install/setup.bash
```

Rebuild after changing Python launch files, Python nodes, or config YAML files.

## Meta Quest Setup

Connect the Quest 3 over USB and make sure ADB can see it:

```bash
adb devices
```

The headset should appear as `device`. If it appears as `unauthorized`, put on the headset and accept the USB debugging prompt.

After launching with the Oculus bridge enabled, check:

```bash
ros2 topic list | grep oculus
```

Expected topics include:

```text
/oculus/left_controller_pose
/oculus/right_controller_pose
/oculus/left_controller_buttons
/oculus/right_controller_buttons
```

To test buttons:

```bash
ros2 topic echo /oculus/left_controller_buttons
```

Press the grip button. `buttons[1]` should change to `1`.

## Simulation Launch

This starts the dual-arm mock ros2_control setup, MoveIt, MoveIt Servo, RViz, the Quest bridge, and both teleop nodes:

```bash
cd /home/amal/franka_ros2_ws
source install/setup.bash

ros2 launch fr3duo_quest_teleop teleop_vr.launch.py \
  launch_oculus_bridge:=true \
  launch_rviz:=true
```

To launch without the Quest bridge:

```bash
ros2 launch fr3duo_quest_teleop teleop_vr.launch.py \
  launch_oculus_bridge:=false \
  launch_rviz:=true
```

The simulation launch uses the top-level `common_base_frame` from `config/teleop_config.yaml` by default. It is set to `mount_link`, the shared mount frame in `fr3_duo_moveit_config`.

You can override it:

```bash
ros2 launch fr3duo_quest_teleop teleop_vr.launch.py \
  common_base_frame:=mount_link \
  launch_oculus_bridge:=true \
  launch_rviz:=true
```

## Hardware Launch

Before hardware teleop:

- Test each FR3 independently with normal Franka bringup.
- Use low gains and low multipliers.
- Keep the emergency stop reachable.
- Start with `launch_oculus_bridge:=false` first.
- Verify both arm controllers are active before moving with Quest.

Launch both real robots:

```bash
cd /home/amal/franka_ros2_ws
source install/setup.bash

ros2 launch fr3duo_quest_teleop hardware_teleop.launch.py \
  left_robot_ip:=192.168.2.102 \
  right_robot_ip:=192.168.2.101
```

Safer first hardware test without Quest:

```bash
ros2 launch fr3duo_quest_teleop hardware_teleop.launch.py \
  left_robot_ip:=<LEFT_FR3_FCI_IP> \
  right_robot_ip:=<RIGHT_FR3_FCI_IP> \
  launch_oculus_bridge:=false
```

The hardware launch uses:

```text
/left/controller_manager
/right/controller_manager
/left/fr3_arm_controller
/right/fr3_arm_controller
/left/servo_node/delta_twist_cmds
/right/servo_node/delta_twist_cmds
/left/franka_gripper/gripper_action
/right/franka_gripper/gripper_action
```

On hardware, each MoveIt Servo node uses the single-arm Franka model, so its
Cartesian command frame must be a link in that arm's model. The defaults are:

```text
left_command_frame:=left_fr3_link0
right_command_frame:=right_fr3_link0
```

Do not use `mount_link` as the hardware Servo command frame unless that link is
part of each single-arm robot model. If Servo logs `Link 'mount_link' not found
in model 'fr3'`, it has crashed and the teleop Twist commands cannot move the
robot.

Check controllers:

```bash
ros2 control list_controllers -c /left/controller_manager
ros2 control list_controllers -c /right/controller_manager
```

Check joint states:

```bash
ros2 topic echo /left/joint_states --once
ros2 topic echo /right/joint_states --once
```

## Command Frames

The simulation launch accepts:

```bash
common_base_frame:=mount_link
```

The default comes from `config/teleop_config.yaml`:

```yaml
common_base_frame: mount_link
```

In simulation, `mount_link` is the shared mount frame of the dual-arm URDF:

```text
base
└── mount_link
    ├── left_base
    │   └── left_fr3v2_link0
    └── right_base
        └── right_fr3v2_link0
```

Verify simulation TF:

```bash
ros2 run tf2_ros tf2_echo mount_link left_fr3v2_link0
ros2 run tf2_ros tf2_echo mount_link right_fr3v2_link0
```

Verify command frame:

```bash
ros2 topic echo /left/servo_node/delta_twist_cmds --once
```

Expected:

```yaml
header:
  frame_id: mount_link
```

For real hardware, Servo command frames are per arm by default:

```bash
left_command_frame:=left_fr3_link0
right_command_frame:=right_fr3_link0
```

These frames must exist in each single-arm MoveIt robot model. Verify hardware
command frames:

```bash
ros2 param get /left/servo_node moveit_servo.planning_frame
ros2 param get /right/servo_node moveit_servo.planning_frame
ros2 topic echo /left/servo_node/delta_twist_cmds --once
```

The left Twist header should be `left_fr3_link0`; the right Twist header should
be `right_fr3_link0`.

## Config Files

### `config/teleop_config.yaml`

Main teleop tuning file. It has one section for `left` and one for `right`.

Important fields:

```yaml
common_base_frame: mount_link
base_frame: mount_link
ee_frame: left_fr3v2_hand_tcp
other_ee_frame: right_fr3v2_hand_tcp
pose_topic: /oculus/left_controller_pose
button_topic: /oculus/left_controller_buttons
twist_topic: /left/servo_node/delta_twist_cmds
gripper_action: /left_hand_controller/gripper_cmd
servo_start_service: /left/servo_node/start_servo
gripper_open_width: 0.08
gripper_closed_width: 0.0
gripper_max_command_width: 0.08
gripper_open_on_start: true
gripper_homing_on_start: false
gripper_trigger_threshold: 0.5
controller_to_robot_rotation: [0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
```

The hardware launch overrides `base_frame`, `ee_frame`, `other_ee_frame`,
`twist_topic`, `servo_start_service`, `gripper_action`,
`gripper_homing_action`, and `gripper_homing_on_start` for each real arm.
By default, hardware teleop uses `left_fr3_link0` and `right_fr3_link0` as the
per-arm base frames.

Motion scaling:

```yaml
linear_multiplier: 0.9
angular_multiplier: 0.0
kp_linear: 8.0
kp_angular: 8.0
```

Meaning:

- `linear_multiplier`: how far the robot target moves for controller movement
- `kp_linear`: how fast the robot chases the target position
- `angular_multiplier`: how much controller rotation affects robot orientation
- `kp_angular`: how fast the robot chases target orientation
- `gripper_trigger_threshold`: trigger value above which a new press toggles
  the gripper between open and closed
- `gripper_open_width`: physical total gap between fingertips for the open
  toggle position, in meters
- `gripper_closed_width`: physical total gap between fingertips for the closed
  toggle position, in meters
- `gripper_max_command_width`: maximum physical total fingertip gap that teleop
  is allowed to command, in meters
- `gripper_open_on_start`: command `gripper_open_width` once when the gripper
  action server becomes available
- `gripper_homing_on_start`: for hardware, home the Franka gripper during
  startup so it opens fully and updates the gripper server's calibrated
  `max_width`

The Franka Hand nominal maximum opening is 0.08 m. The hardware gripper server
still validates against its runtime calibrated `max_width`; if it reports a much
smaller value, home the gripper and check for mechanical limits or mounted
fingers that reduce the usable travel.

To disable rotation and tune linear motion only:

```yaml
angular_multiplier: 0.0
```

Controller-to-robot rotation:

```yaml
controller_to_robot_rotation: [0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
```

This is a row-major 3x3 matrix:

```text
[r00, r01, r02,
 r10, r11, r12,
 r20, r21, r22]
```

The teleop node applies it to Quest controller deltas before computing the
robot target:

```text
robot_delta = controller_to_robot_rotation * quest_delta
```

The default matrix preserves the old linear behavior:

```text
robot base X/forward <- Quest z * -1
robot base Y/left    <- Quest x * -1
robot base Z/up      <- Quest y *  1
```

The same matrix is also applied to controller rotation vectors before
orientation tracking. If each robot is mounted at a different angle relative to
the Meta Quest/control coordinate system, set a different matrix in the `left`
and `right` sections.

```yaml
left:
  controller_to_robot_rotation: [...]

right:
  controller_to_robot_rotation: [...]
```

Safety distance:

```yaml
collision_distance: 0.20
```

If the two end effectors are closer than this, the teleop node publishes zero Twist for that arm.

### `config/fr3_servo_config.yaml`

MoveIt Servo configuration shared by both arms.

Important fields:

```yaml
command_in_type: "speed_units"
cartesian_command_in_topic: ~/delta_twist_cmds
command_out_type: trajectory_msgs/JointTrajectory
publish_period: 0.02
joint_topic: /joint_states
check_collisions: true
```

Usually you do not need to edit this during normal teleop tuning. Tune `teleop_config.yaml` first.

### `config/hardware_ros2_controllers.yaml`

Controller configuration used by `hardware_teleop.launch.py`.

It defines namespaced real-hardware arm controllers:

```text
/left/fr3_arm_controller
/right/fr3_arm_controller
```

Only edit this if you are changing controller types, joint names, or real hardware control interfaces.

## Runtime Checks

Check teleop node frames:

```bash
ros2 param get /left/franka_teleop_node base_frame
ros2 param get /right/franka_teleop_node base_frame
```

Check Servo frames:

```bash
ros2 param get /left/servo_node moveit_servo.planning_frame
ros2 param get /left/servo_node moveit_servo.robot_link_command_frame
```

Check Twist commands:

```bash
ros2 topic echo /left/servo_node/delta_twist_cmds
```

Hold the Quest grip and move the controller. Twist values should become non-zero. Release grip and they should return to zero.

Check Servo status:

```bash
ros2 topic echo /left/servo_node/status
ros2 topic echo /right/servo_node/status
```

## Notes

The warning below is expected if you are not using a depth camera or octomap sensor:

```text
No 3D sensor plugin(s) defined for octomap updates
```

It is not the reason teleop fails.

If launch behaves strangely after config changes, stop old ROS processes, rebuild, source, and relaunch:

```bash
pkill -f franka_teleop_node
pkill -f servo_node_main

cd /home/amal/franka_ros2_ws
colcon build --packages-select fr3duo_quest_teleop --symlink-install
source install/setup.bash
```
