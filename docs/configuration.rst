Configuration
=============

``config/teleop_config.yaml``
-----------------------------

Main teleop tuning file. It has one section for ``left`` and one for ``right``.
Keep this file focused on values you normally tune during teleoperation.

Current shape:

.. code-block:: yaml

   common_base_frame: mount_link

   left:
     arm_side: left
     linear_multiplier: 0.9
     angular_multiplier: 0.9
     kp_linear: 20.0
     kp_angular: 20.0
     collision_distance: 0.35
     controller_to_robot_rotation: [0.440, 0.174, -0.881, -0.500, -0.767, -0.401, -0.746, 0.617, -0.251]

   right:
     arm_side: right
     linear_multiplier: 0.9
     angular_multiplier: 0.9
     kp_linear: 8.0
     kp_angular: 8.0
     collision_distance: 0.20
     controller_to_robot_rotation: [-0.440, 0.174, -0.881, -0.500, 0.767, 0.401, 0.746, 0.617, -0.251]

The launch files and node defaults provide the usual topics, frames, gripper
actions, Servo start services, and gripper limits. Hardware launch still
overrides the real-robot frames, gripper actions, homing behavior, and
conservative hardware gains.

``arm_side``
   Selects the controller/arm side for one teleop node. It also lets the node
   infer default Quest topics, Servo topics, gripper actions, and simulation
   end-effector frames.

Motion Scaling
--------------

.. code-block:: yaml

   linear_multiplier: 0.9
   angular_multiplier: 0.0
   kp_linear: 8.0
   kp_angular: 8.0

``linear_multiplier``
   How far the robot target moves for controller movement.

``kp_linear``
   How fast the robot chases the target position.

``angular_multiplier``
   How much controller rotation affects robot orientation.

``kp_angular``
   How fast the robot chases target orientation.

To disable rotation and tune linear motion only:

.. code-block:: yaml

   angular_multiplier: 0.0

Advanced Gripper Parameters
---------------------------

The gripper values are no longer duplicated in ``teleop_config.yaml`` because
the defaults match normal Franka Hand use:

``gripper_trigger_threshold``
   Trigger value above which a new press toggles the gripper between open and
   closed.

``gripper_open_width``
   Physical total gap between fingertips for the open toggle position, in
   meters.

``gripper_closed_width``
   Physical total gap between fingertips for the closed toggle position, in
   meters.

``gripper_max_command_width``
   Maximum physical total fingertip gap that teleop is allowed to command, in
   meters.

``gripper_open_on_start``
   Command ``gripper_open_width`` once when the gripper action server becomes
   available.

``gripper_homing_on_start``
   For hardware, home the Franka gripper during startup so it opens fully and
   updates the gripper server's calibrated ``max_width``.

The Franka Hand nominal maximum opening is 0.08 m. The hardware gripper server
still validates against its runtime calibrated ``max_width``. If it reports a
much smaller value, home the gripper and check for mechanical limits or mounted
fingers that reduce usable travel.

If a setup needs different gripper values, pass these parameters from launch or
add only the changed values back under the affected arm.

Controller-to-Robot Rotation
----------------------------

.. code-block:: yaml

   controller_to_robot_rotation: [0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0]

This is a row-major 3x3 matrix:

.. code-block:: text

   [r00, r01, r02,
    r10, r11, r12,
    r20, r21, r22]

The teleop node applies it to Quest controller deltas before computing the robot
target:

.. code-block:: text

   robot_delta = controller_to_robot_rotation * quest_delta

The default matrix preserves the old linear behavior:

.. code-block:: text

   robot base X/forward <- Quest z * -1
   robot base Y/left    <- Quest x * -1
   robot base Z/up      <- Quest y *  1

The same matrix is also applied to controller rotation vectors before
orientation tracking. If each robot is mounted at a different angle relative to
the Meta Quest/control coordinate system, set a different matrix in the ``left``
and ``right`` sections.

.. code-block:: yaml

   left:
     controller_to_robot_rotation: [...]

   right:
     controller_to_robot_rotation: [...]

Safety Distance
---------------

.. code-block:: yaml

   collision_distance: 0.20

If the two end effectors are closer than this, the teleop node publishes zero
Twist for that arm.

``config/fr3_servo_config.yaml``
--------------------------------

MoveIt Servo configuration shared by both arms.

Important fields:

.. code-block:: yaml

   command_in_type: "speed_units"
   cartesian_command_in_topic: ~/delta_twist_cmds
   command_out_type: trajectory_msgs/JointTrajectory
   publish_period: 0.02
   joint_topic: /joint_states
   check_collisions: true

Usually you do not need to edit this during normal teleop tuning. Tune
``teleop_config.yaml`` first.

``config/hardware_ros2_controllers.yaml``
-----------------------------------------

Controller configuration used by ``hardware_teleop.launch.py``. It defines
namespaced real-hardware arm controllers:

.. code-block:: text

   /left/fr3_arm_controller
   /right/fr3_arm_controller

Only edit this if you are changing controller types, joint names, or real
hardware control interfaces.
