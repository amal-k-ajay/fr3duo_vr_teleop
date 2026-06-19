Runtime Checks
==============

Check teleop node frames:

.. code-block:: bash

   ros2 param get /left/franka_teleop_node base_frame
   ros2 param get /right/franka_teleop_node base_frame

Check Servo frames:

.. code-block:: bash

   ros2 param get /left/servo_node moveit_servo.planning_frame
   ros2 param get /left/servo_node moveit_servo.robot_link_command_frame

Check Twist commands:

.. code-block:: bash

   ros2 topic echo /left/servo_node/delta_twist_cmds

Hold the Quest grip and move the controller. Twist values should become
non-zero. Release grip and they should return to zero.

Check Servo status:

.. code-block:: bash

   ros2 topic echo /left/servo_node/status
   ros2 topic echo /right/servo_node/status

Expected Warning
----------------

The warning below is expected if you are not using a depth camera or octomap
sensor:

.. code-block:: text

   No 3D sensor plugin(s) defined for octomap updates

It is not the reason teleop fails.

Restart After Config Changes
----------------------------

If launch behaves strangely after config changes, stop old ROS processes,
rebuild, source, and relaunch:

.. code-block:: bash

   pkill -f franka_teleop_node
   pkill -f servo_node_main

   cd /home/amal/franka_ros2_ws
   colcon build --packages-select fr3duo_quest_teleop --symlink-install
   source install/setup.bash
