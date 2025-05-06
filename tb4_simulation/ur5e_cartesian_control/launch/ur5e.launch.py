from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

# argument
    declared_arguments = [
        DeclareLaunchArgument("use_fake", default_value="true"),
        DeclareLaunchArgument("use_gazebo", default_value="false"),
        DeclareLaunchArgument("x", default_value="0.2"),
        DeclareLaunchArgument("y", default_value="0.2"),
        DeclareLaunchArgument("z", default_value="0.3")
    ]

    use_fake = LaunchConfiguration("use_fake")
    use_gazebo = LaunchConfiguration("use_gazebo")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

# path
    controllers_file = os.path.join(
        get_package_share_directory('ur5e_cartesian_control'),
        'config',
        'ur5e_controller.yaml'
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare("ur5e_cartesian_control"),
        "config",
        "config.rviz"
    ])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("ur5e_cartesian_control"),
            "urdf",
            "ur5e_cartesian.urdf.xacro"
        ]),
        " ",
        "sim_ignition:=", use_gazebo, " ",
        "use_fake_hardware:=", use_fake
    ])

    robot_description = {"robot_description": robot_description_content}

    world_path = os.path.join(
        get_package_share_directory('ur5e_cartesian_control'),
        'config',
        'tot.world'
    )
    urdf_path = os.path.join(
        get_package_share_directory('ur5e_cartesian_control'),
        'urdf',
        'ur5e_cartesian.urdf'
    )

# controller
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file],
        output="screen",
        condition = IfCondition(use_fake)
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen"
    )

    joint_vel_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_velocities", "-c", "/controller_manager"],
        output="screen"
    )

    cartesian_controller_node = Node(
        package="ros2_controllers_cartesian",
        executable="cartesian_controller_node_exe",
        name="cartesian_controller_node",
        output="screen",
        parameters=[
            robot_description,
            {"sensor_base_frame": "sensor_base"},
            {"tool_frame":        "tool0"},
            {"target_frame":      "target"},
            {"robot_base_frame":  "base_link"},
            {"flange_frame":      "tool0"},
            {"joint_names":       [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint"
            ]},
            {"control_rate":      10.0},
            {"error_gain":        1.0},
            {"velocity_command_topic": "/joint_velocities/commands"}
        ]
    )
# tf
    tf_tool = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=['-0.5', '0.5', '0.5', "0", "0", "0", "sensor_base", "tool"],
        name="tf_tool",
        output="screen"
    )

    tf_sensor_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "sensor_base"],
        name="tf_sensor_base",
        output="screen"
    )

    tf_target = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[x, y, z, "0", "0", "0", "sensor_base", "target"],
        name="tf_target",
        output="screen"
    )
# sender and rviz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen"
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

# gazebo
    generate_urdf_cmd = ExecuteProcess(
        cmd=[
            FindExecutable(name="xacro"),
            PathJoinSubstitution([
            FindPackageShare("ur5e_cartesian_control"),
            "urdf",
            "ur5e_cartesian.urdf.xacro"
    ]),
            '-o',
            PathJoinSubstitution([
            FindPackageShare("ur5e_cartesian_control"),
            "urdf",
            "ur5e_cartesian.urdf"
    ])
        ],
        output='screen',
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ros_ign_gazebo'),
            '/launch/ign_gazebo.launch.py'
        ]),
        launch_arguments={'ign_args': f'-r {world_path}'}.items(),
        condition=IfCondition(use_gazebo)
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', urdf_path,
            '-entity', 'ur5e',
            '-x', '0', '-y', '0', '-z', '0.05'
        ],
        condition=IfCondition(use_gazebo)
    )

    return LaunchDescription(declared_arguments + [
        generate_urdf_cmd,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster,
        joint_vel_controller,
        cartesian_controller_node,
        tf_tool,
        tf_sensor_base,
        tf_target,
        sender_robot_node,
        sender_target_node,
        gazebo_launch,
        spawn_robot,
        rviz
    ])
