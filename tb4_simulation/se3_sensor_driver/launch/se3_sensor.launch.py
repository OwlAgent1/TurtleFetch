from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory('se3_sensor_driver'),
        'config',
        'se3_sensor.yaml'
    )

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("se3_sensor_driver"),
            "urdf",
            "se3_sensor.urdf.xacro"
        ])
    ])

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, config_file],
        output='screen'
    )

    spawn_robot_sensor = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'robot_broadcaster'],
        output='screen'
    )

    spawn_target_sensor = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'target_broadcaster'],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', get_package_share_directory('se3_sensor_driver') + '/rviz/config.rviz']
    )

    tool_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_tool',
        arguments=['1', '-1', '1', '0', '0', '0', 'sensor_base', 'tool'],
        output='screen'
    )

    target_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_target',
        arguments=['1', '1', '1', '0', '0', '0', 'sensor_base', 'target'],
        output='screen'
    )

    sender_robot_node = Node(
        package='se3_sensor_driver',
        executable='sensor_tf_sender',
        name='sensor_tf_sender_robot',
        output='screen',
        parameters=[
            {'tool_frame': 'tool'},
            {'port': 12345},
            {'sensor_frame': 'sensor_base'}
        ]
    )

    sender_target_node = Node(
        package='se3_sensor_driver',
        executable='sensor_tf_sender',
        name='sensor_tf_sender_target',
        output='screen',
        parameters=[
            {'tool_frame': 'target'},
            {'port': 12346},
            {'sensor_frame': 'sensor_base'}
        ]
    )
    sensorbase_to_baselink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_sensorbase',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'sensor_base'],
    output='screen'
    )
    return LaunchDescription([
        controller_manager_node,
        spawn_robot_sensor,
        spawn_target_sensor,
        rviz_node,
        robot_state_publisher_node,
        tool_static_tf,
        target_static_tf,
        sender_robot_node,   
        sender_target_node,
        sensorbase_to_baselink
    ])
