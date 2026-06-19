User Guide
==========

Build
-----

Build from the workspace root:

.. code-block:: bash

   cd /home/amal/franka_ros2_ws
   colcon build --packages-select fr3duo_quest_teleop --symlink-install
   source install/setup.bash

Rebuild after changing Python launch files, Python nodes, or config YAML files.

Meta Quest Setup
----------------

Connect the Quest 3 over USB and make sure ADB can see it:

.. code-block:: bash

   adb devices

The headset should appear as ``device``. If it appears as ``unauthorized``, put
on the headset and accept the USB debugging prompt.

After launching with the Oculus bridge enabled, check:

.. code-block:: bash

   ros2 topic list | grep oculus

Expected topics include:

.. code-block:: text

   /oculus/left_controller_pose
   /oculus/right_controller_pose
   /oculus/left_controller_buttons
   /oculus/right_controller_buttons

To test buttons:

.. code-block:: bash

   ros2 topic echo /oculus/left_controller_buttons

Press the grip button. ``buttons[1]`` should change to ``1``.

Simulation Launch
-----------------

This starts the dual-arm mock ``ros2_control`` setup, MoveIt, MoveIt Servo,
RViz, the Quest bridge, and both teleop nodes:

.. code-block:: bash

   cd /home/amal/franka_ros2_ws
   source install/setup.bash

   ros2 launch fr3duo_quest_teleop teleop_vr.launch.py \
     launch_oculus_bridge:=true \
     launch_rviz:=true

To launch without the Quest bridge:

.. code-block:: bash

   ros2 launch fr3duo_quest_teleop teleop_vr.launch.py \
     launch_oculus_bridge:=false \
     launch_rviz:=true

The simulation launch uses the top-level ``common_base_frame`` from
``config/teleop_config.yaml`` by default. It is set to ``mount_link``, the
shared mount frame in ``fr3_duo_moveit_config``.

Override it with:

.. code-block:: bash

   ros2 launch fr3duo_quest_teleop teleop_vr.launch.py \
     common_base_frame:=mount_link \
     launch_oculus_bridge:=true \
     launch_rviz:=true

Hardware Launch
---------------

Before hardware teleop:

* Use low gains and low multipliers.
* Keep the emergency stop reachable.
* Start with ``launch_oculus_bridge:=false`` first.
* Verify both arm controllers are active before moving with Quest.

Launch both real robots:

.. code-block:: bash

   cd /home/amal/franka_ros2_ws
   source install/setup.bash

   ros2 launch fr3duo_quest_teleop hardware_teleop.launch.py \
     left_robot_ip:=<LEFT_FR3_FCI_IP> \
     right_robot_ip:=<RIGHT_FR3_FCI_IP>

Safer hardware test without Quest:

.. code-block:: bash

   ros2 launch fr3duo_quest_teleop hardware_teleop.launch.py \
     left_robot_ip:=<LEFT_FR3_FCI_IP> \
     right_robot_ip:=<RIGHT_FR3_FCI_IP> \
     launch_oculus_bridge:=false

The hardware launch uses:

.. code-block:: text

   /left/controller_manager
   /right/controller_manager
   /left/fr3_arm_controller
   /right/fr3_arm_controller
   /left/servo_node/delta_twist_cmds
   /right/servo_node/delta_twist_cmds
   /left/franka_gripper/gripper_action
   /right/franka_gripper/gripper_action

On hardware, each MoveIt Servo node uses the single-arm Franka model, so its
Cartesian command frame must be a link in that arm's model. The defaults are:

.. code-block:: text

   left_command_frame:=left_fr3_link0
   right_command_frame:=right_fr3_link0

Check controllers:

.. code-block:: bash

   ros2 control list_controllers -c /left/controller_manager
   ros2 control list_controllers -c /right/controller_manager

Check joint states:

.. code-block:: bash

   ros2 topic echo /left/joint_states --once
   ros2 topic echo /right/joint_states --once

Command Frames
--------------

The simulation launch accepts:

.. code-block:: bash

   common_base_frame:=mount_link

The default comes from ``config/teleop_config.yaml``:

.. code-block:: yaml

   common_base_frame: mount_link

In simulation, ``mount_link`` is the shared mount frame of the dual-arm URDF:

.. code-block:: text

   base
   `-- mount_link
       |-- left_base
       |   `-- left_fr3v2_link0
       `-- right_base
           `-- right_fr3v2_link0

Verify simulation TF:

.. code-block:: bash

   ros2 run tf2_ros tf2_echo mount_link left_fr3v2_link0
   ros2 run tf2_ros tf2_echo mount_link right_fr3v2_link0

Verify command frame:

.. code-block:: bash

   ros2 topic echo /left/servo_node/delta_twist_cmds --once

Expected:

.. code-block:: yaml

   header:
     frame_id: mount_link

For real hardware, Servo command frames are per arm by default:

.. code-block:: bash

   left_command_frame:=left_fr3_link0
   right_command_frame:=right_fr3_link0

These frames must exist in each single-arm MoveIt robot model. Verify hardware
command frames:

.. code-block:: bash

   ros2 param get /left/servo_node moveit_servo.planning_frame
   ros2 param get /right/servo_node moveit_servo.planning_frame
   ros2 topic echo /left/servo_node/delta_twist_cmds --once

The left Twist header should be ``left_fr3_link0``; the right Twist header
should be ``right_fr3_link0``.
