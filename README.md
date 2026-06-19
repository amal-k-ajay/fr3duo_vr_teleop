# FR3 Duo Quest Teleop

ROS 2 package for teleoperating a dual Franka FR3 setup with Meta Quest 3
controllers.

The package provides:

- simulation/RViz teleop through `fr3_duo_moveit_config`
- real hardware teleop through `franka_bringup`
- a Quest bridge that publishes controller poses and buttons
- one teleop node per arm for Cartesian MoveIt Servo commands and gripper
  toggles

Grip is the deadman switch. While grip is held, controller motion is mapped to
Cartesian end-effector motion. Trigger toggles the gripper.

## Build

From the workspace root:

```bash
cd /home/amal/franka_ros2_ws
colcon build --packages-select fr3duo_quest_teleop --symlink-install
source install/setup.bash
```

Rebuild after changing launch files, Python nodes, or config YAML files.

## Quick Start

Simulation with Quest bridge and RViz:

```bash
ros2 launch fr3duo_quest_teleop teleop_vr.launch.py \
  launch_oculus_bridge:=true \
  launch_rviz:=true
```

Simulation without the Quest bridge:

```bash
ros2 launch fr3duo_quest_teleop teleop_vr.launch.py \
  launch_oculus_bridge:=false \
  launch_rviz:=true
```

Hardware:

```bash
ros2 launch fr3duo_quest_teleop hardware_teleop.launch.py \
  left_robot_ip:=<LEFT_FR3_FCI_IP> \
  right_robot_ip:=<RIGHT_FR3_FCI_IP>
```

For a safer first hardware check, start without the Quest bridge:

```bash
ros2 launch fr3duo_quest_teleop hardware_teleop.launch.py \
  left_robot_ip:=<LEFT_FR3_FCI_IP> \
  right_robot_ip:=<RIGHT_FR3_FCI_IP> \
  launch_oculus_bridge:=false
```

## Quest Check

Connect the headset over USB and verify ADB:

```bash
adb devices
```

After launching the bridge, expected ROS topics include:

```text
/oculus/left_controller_pose
/oculus/right_controller_pose
/oculus/left_controller_buttons
/oculus/right_controller_buttons
```

## Safety

Before hardware teleop:

- keep the emergency stop reachable
- start with low gains and low multipliers
- verify both arm controllers are active
- verify frames and Twist commands before holding grip

## Documentation

Detailed setup, launch arguments, command frames, config tuning, runtime checks,
and Python API documentation live in the Sphinx docs:

```bash
cd docs
make html
```

Open `docs/_build/html/index.html`.
