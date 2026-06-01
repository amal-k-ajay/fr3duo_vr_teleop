# Commands for launching meta quest teleop

## Sim launch

cd /home/amal/franka_ros2_ws
source install/setup.bash

ros2 launch fr3duo_quest_teleop teleop_vr.launch.py launch_oculus_bridge:=true launch_rviz:=true

## Hardware launch

cd /home/amal/franka_ros2_ws
source install/setup.bash

ros2 launch fr3duo_quest_teleop hardware_teleop.launch.py \
  left_robot_ip:=<LEFT_FR3_FCI_IP> \
  right_robot_ip:=<RIGHT_FR3_FCI_IP>


### Testing without Oculus

ros2 launch fr3duo_quest_teleop hardware_teleop.launch.py \
  left_robot_ip:=<LEFT_FR3_FCI_IP> \
  right_robot_ip:=<RIGHT_FR3_FCI_IP> \
  launch_oculus_bridge:=false