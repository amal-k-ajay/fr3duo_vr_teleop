import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)


def load_prefixed_kinematics(prefix):
    kinematics = load_yaml(
        'franka_fr3_moveit_config',
        'config/kinematics.yaml',
    )
    return {f'{prefix}_fr3_arm': kinematics['fr3_arm']}


def robot_description_for_arm(prefix, robot_ip, load_gripper):
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        'fr3',
        'fr3.urdf.xacro',
    )
    robot_description_config = Command([
        FindExecutable(name='xacro'),
        ' ',
        franka_xacro_file,
        ' ros2_control:=false',
        ' robot_type:=fr3',
        ' arm_prefix:=', prefix,
        ' hand:=', load_gripper,
        ' robot_ip:=', robot_ip,
        ' use_fake_hardware:=false',
        ' fake_sensor_commands:=false',
    ])
    return {
        'robot_description': ParameterValue(
            robot_description_config,
            value_type=str,
        )
    }


def robot_description_semantic_for_arm(prefix, load_gripper):
    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        'fr3',
        'fr3.srdf.xacro',
    )
    robot_description_semantic_config = Command([
        FindExecutable(name='xacro'),
        ' ',
        franka_semantic_xacro_file,
        ' robot_type:=fr3',
        ' arm_prefix:=', prefix,
        ' hand:=', load_gripper,
    ])
    return {
        'robot_description_semantic': ParameterValue(
            robot_description_semantic_config,
            value_type=str,
        )
    }


def servo_params_for_arm(
    common_servo_config,
    prefix,
    namespace,
    command_frame,
):
    params = dict(common_servo_config)
    params.update({
        'move_group_name': f'{prefix}_fr3_arm',
        'planning_frame': command_frame,
        'ee_frame_name': f'{prefix}_fr3_hand_tcp',
        'robot_link_command_frame': command_frame,
        'command_out_topic': (
            f'/{namespace}/fr3_arm_controller/joint_trajectory'
        ),
        'joint_topic': f'/{namespace}/joint_states',
    })
    return {'moveit_servo': params}


def teleop_params_for_arm(
    base_config,
    side,
    prefix,
    namespace,
    other_prefix,
    command_frame,
):
    params = dict(base_config[side])
    params.update({
        'base_frame': command_frame,
        'ee_frame': f'{prefix}_fr3_hand_tcp',
        'other_ee_frame': f'{other_prefix}_fr3_hand_tcp',
        'twist_topic': f'/{namespace}/servo_node/delta_twist_cmds',
        'servo_start_service': f'/{namespace}/servo_node/start_servo',
        'gripper_action': f'/{namespace}/franka_gripper/gripper_action',
        # Conservative defaults for first real-robot tests.
        'linear_multiplier': 0.7,
        'angular_multiplier': 0.7,
        'kp_linear': 0.6,
        'kp_angular': 0.6,
        'collision_distance': 0.35,
    })
    return params


def generate_launch_description():
    teleop_pkg = 'fr3duo_quest_teleop'

    left_robot_ip = LaunchConfiguration('left_robot_ip')
    right_robot_ip = LaunchConfiguration('right_robot_ip')
    launch_oculus_bridge = LaunchConfiguration('launch_oculus_bridge')
    load_gripper = LaunchConfiguration('load_gripper')
    teleop_startup_delay = LaunchConfiguration('teleop_startup_delay')
    left_command_frame = LaunchConfiguration('left_command_frame')
    right_command_frame = LaunchConfiguration('right_command_frame')

    controllers_yaml = os.path.join(
        get_package_share_directory(teleop_pkg),
        'config',
        'hardware_ros2_controllers.yaml',
    )
    common_servo_config = load_yaml(teleop_pkg, 'config/fr3_servo_config.yaml')
    teleop_config = load_yaml(teleop_pkg, 'config/teleop_config.yaml')
    left_robot_description = robot_description_for_arm(
        'left',
        left_robot_ip,
        load_gripper,
    )
    right_robot_description = robot_description_for_arm(
        'right',
        right_robot_ip,
        load_gripper,
    )
    left_robot_description_semantic = robot_description_semantic_for_arm(
        'left',
        load_gripper,
    )
    right_robot_description_semantic = robot_description_semantic_for_arm(
        'right',
        load_gripper,
    )
    left_kinematics = load_prefixed_kinematics('left')
    right_kinematics = load_prefixed_kinematics('right')

    left_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('franka_bringup'),
                'launch',
                'franka.launch.py',
            ])
        ),
        launch_arguments={
            'robot_type': 'fr3',
            'robot_ip': left_robot_ip,
            'namespace': 'left',
            'arm_prefix': 'left',
            'load_gripper': load_gripper,
            'use_fake_hardware': 'false',
            'fake_sensor_commands': 'false',
            'controllers_yaml': controllers_yaml,
        }.items(),
    )

    right_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('franka_bringup'),
                'launch',
                'franka.launch.py',
            ])
        ),
        launch_arguments={
            'robot_type': 'fr3',
            'robot_ip': right_robot_ip,
            'namespace': 'right',
            'arm_prefix': 'right',
            'load_gripper': load_gripper,
            'use_fake_hardware': 'false',
            'fake_sensor_commands': 'false',
            'controllers_yaml': controllers_yaml,
        }.items(),
    )

    left_arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace='left',
        arguments=[
            'fr3_arm_controller',
            '--controller-manager',
            '/left/controller_manager',
            '--controller-manager-timeout',
            '60',
        ],
        output='screen',
    )

    right_arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace='right',
        arguments=[
            'fr3_arm_controller',
            '--controller-manager',
            '/right/controller_manager',
            '--controller-manager-timeout',
            '60',
        ],
        output='screen',
    )

    left_servo = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        namespace='left',
        output='screen',
        parameters=[
            servo_params_for_arm(
                common_servo_config,
                'left',
                'left',
                left_command_frame,
            ),
            left_robot_description,
            left_robot_description_semantic,
            left_kinematics,
        ],
    )

    right_servo = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        namespace='right',
        output='screen',
        parameters=[
            servo_params_for_arm(
                common_servo_config,
                'right',
                'right',
                right_command_frame,
            ),
            right_robot_description,
            right_robot_description_semantic,
            right_kinematics,
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
        parameters=[teleop_params_for_arm(
            teleop_config,
            'left',
            'left',
            'left',
            'right',
            left_command_frame,
        )],
    )

    right_teleop = Node(
        package=teleop_pkg,
        executable='franka_teleop_node',
        name='franka_teleop_node',
        namespace='right',
        output='screen',
        parameters=[teleop_params_for_arm(
            teleop_config,
            'right',
            'right',
            'right',
            'left',
            right_command_frame,
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'left_robot_ip',
            description='FCI IP address or hostname for the left FR3',
        ),
        DeclareLaunchArgument(
            'right_robot_ip',
            description='FCI IP address or hostname for the right FR3',
        ),
        DeclareLaunchArgument(
            'load_gripper',
            default_value='true',
            description='Launch the Franka gripper action servers',
        ),
        DeclareLaunchArgument(
            'launch_oculus_bridge',
            default_value='true',
            description='Connect to the Meta Quest through oculus_reader',
        ),
        DeclareLaunchArgument(
            'teleop_startup_delay',
            default_value='25.0',
            description=(
                'Seconds to wait before starting Servo and Quest teleop nodes'
            ),
        ),
        DeclareLaunchArgument(
            'left_command_frame',
            default_value='left_fr3_link0',
            description=(
                'Left hardware frame used for Cartesian Servo commands'
            ),
        ),
        DeclareLaunchArgument(
            'right_command_frame',
            default_value='right_fr3_link0',
            description=(
                'Right hardware frame used for Cartesian Servo commands'
            ),
        ),
        left_bringup,
        right_bringup,
        TimerAction(
            period=8.0,
            actions=[left_arm_spawner, right_arm_spawner],
        ),
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
