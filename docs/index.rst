FR3 Duo Quest Teleop
====================

``fr3duo_quest_teleop`` is a ROS 2 package for teleoperating a dual Franka FR3
setup with Meta Quest controllers.

Grip is used as the deadman switch. While grip is held, controller translation
is mapped to Cartesian end-effector motion through MoveIt Servo. Trigger toggles
the gripper.

.. toctree::
   :maxdepth: 2
   :caption: User Guide

   user_guide
   configuration
   runtime_checks

.. toctree::
   :maxdepth: 2
   :caption: Python API

   api
