import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)


def load_text(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as file:
        return file.read()


def servo_params_for_side(common_servo_config, side, command_frame):
    if side == 'left':
        prefix = 'left_fr3v2'
        move_group_name = 'left_arm'
        command_out_topic = '/left_arm_controller/joint_trajectory'
    else:
        prefix = 'right_fr3v2'
        move_group_name = 'right_arm'
        command_out_topic = '/right_arm_controller/joint_trajectory'

    params = dict(common_servo_config)
    params.update({
        'move_group_name': move_group_name,
        'planning_frame': command_frame,
        'ee_frame_name': f'{prefix}_hand_tcp',
        'robot_link_command_frame': command_frame,
        'command_out_topic': command_out_topic,
    })
    return {'moveit_servo': params}


def generate_launch_description():
    moveit_pkg = 'fr3_duo_moveit_config'
    teleop_pkg = 'fr3duo_quest_teleop'

    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_moveit = LaunchConfiguration('launch_moveit')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_oculus_bridge = LaunchConfiguration('launch_oculus_bridge')
    teleop_startup_delay = LaunchConfiguration('teleop_startup_delay')
    common_base_frame = LaunchConfiguration('common_base_frame')

    robot_description = {
        'robot_description': load_text(moveit_pkg, 'config/fr3_duo.urdf')
    }
    robot_description_semantic = {
        'robot_description_semantic': load_text(moveit_pkg, 'config/fr3_duo.srdf')
    }
    kinematics_config = load_yaml(moveit_pkg, 'config/kinematics.yaml')
    common_servo_config = load_yaml(teleop_pkg, 'config/fr3_servo_config.yaml')
    teleop_config = load_yaml(teleop_pkg, 'config/teleop_config.yaml')

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(moveit_pkg),
                'launch',
                'demo.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'launch_rviz': launch_rviz,
        }.items(),
        condition=IfCondition(launch_moveit),
    )

    left_servo = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        namespace='left',
        output='screen',
        parameters=[
            servo_params_for_side(common_servo_config, 'left', common_base_frame),
            robot_description,
            robot_description_semantic,
            kinematics_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    right_servo = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        namespace='right',
        output='screen',
        parameters=[
            servo_params_for_side(common_servo_config, 'right', common_base_frame),
            robot_description,
            robot_description_semantic,
            kinematics_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    oculus_bridge = Node(
        package=teleop_pkg,
        executable='oculus_bridge_node',
        name='oculus_bridge_node',
        output='screen',
        condition=IfCondition(launch_oculus_bridge),
    )

    left_teleop = Node(
        package=teleop_pkg,
        executable='franka_teleop_node',
        name='franka_teleop_node',
        namespace='left',
        output='screen',
        parameters=[teleop_config['left'], {'base_frame': common_base_frame}],
    )

    right_teleop = Node(
        package=teleop_pkg,
        executable='franka_teleop_node',
        name='franka_teleop_node',
        namespace='right',
        output='screen',
        parameters=[teleop_config['right'], {'base_frame': common_base_frame}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'launch_moveit',
            default_value='true',
            description='Launch the fr3_duo MoveIt/ros2_control demo stack',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Launch RViz through the fr3_duo MoveIt demo launch',
        ),
        DeclareLaunchArgument(
            'launch_oculus_bridge',
            default_value='true',
            description='Connect to the Meta Quest through oculus_reader',
        ),
        DeclareLaunchArgument(
            'teleop_startup_delay',
            default_value='20.0',
            description='Seconds to wait before starting Servo and Quest teleop nodes',
        ),
        DeclareLaunchArgument(
            'common_base_frame',
            default_value='base',
            description='Shared frame used for both arms Cartesian teleop commands',
        ),
        moveit_launch,
        TimerAction(
            period=teleop_startup_delay,
            actions=[
                left_servo,
                right_servo,
                oculus_bridge,
                left_teleop,
                right_teleop,
            ],
        ),
    ])
