# FR3 Duo VR Teleop

ROS 2 package for teleoperating a Franka FR3 Duo robot with Meta Quest 3
controllers.

The package provides:

- simulation/RViz teleop through `fr3_duo_moveit_config`
- real hardware teleop through `franka_bringup`
- a Quest bridge that publishes controller poses and buttons
- one teleop node per arm for Cartesian MoveIt Servo commands and gripper
  toggles

Grip is the deadman switch. While grip is held, controller motion is mapped to
Cartesian end-effector motion. Trigger toggles the gripper.

## Oculus Reader Dependency

The Quest bridge depends on
[rail-berkeley/oculus_reader](https://github.com/rail-berkeley/oculus_reader).
Clone and install it inside this repository directory so the package layout is:

```text
franka_ros2_ws/src/fr3duo_quest_teleop/
├── fr3duo_quest_teleop/
├── launch/
├── config/
└── oculus_reader/
```

From the parent folder of this README:

```bash
git clone https://github.com/rail-berkeley/oculus_reader.git oculus_reader
python3 -m pip install -r oculus_reader/requirements.txt
python3 -m pip install -e oculus_reader
```

The `oculus_bridge_node` imports `oculus_reader` from this local folder at
runtime. If `oculus_reader` is cloned somewhere else, the Quest bridge will not
start unless the import path is updated.

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
