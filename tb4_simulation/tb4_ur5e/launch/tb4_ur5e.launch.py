from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from irobot_create_common_bringup.namespace import GetNamespacedName
from irobot_create_common_bringup.offset import OffsetParser, RotationalOffsetX, RotationalOffsetY
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():

    ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    DeclareLaunchArgument('namespace', default_value='tb4',
                          description='Robot namespace'),
    DeclareLaunchArgument('localization', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to launch localization'),
    DeclareLaunchArgument('slam', default_value='true',
                          choices=['true', 'false'],
                          description='Whether to launch SLAM'),
    DeclareLaunchArgument('nav2', default_value='true',
                          choices=['true', 'false'],
                          description='Whether to launch Nav2'),
    DeclareLaunchArgument('world', default_value='wyman_ur5e',
                          description='Ignition World'),
    DeclareLaunchArgument("x_ur", default_value="0.2"),
    DeclareLaunchArgument("y_ur", default_value="0.2"),
    DeclareLaunchArgument("z_ur", default_value="0.3"),
    DeclareLaunchArgument("use_fake", default_value="false"),
    DeclareLaunchArgument("use_gazebo", default_value="true"),
    ]

    # Directories
    pkg_turtlebot4_ignition_bringup = get_package_share_directory('turtlebot4_ignition_bringup')
    pkg_turtlebot4_description = get_package_share_directory('turtlebot4_description')
    pkg_turtlebot4_viz = get_package_share_directory('turtlebot4_viz')
    pkg_turtlebot4_navigation = get_package_share_directory('turtlebot4_navigation')
    pkg_irobot_create_common_bringup = get_package_share_directory('irobot_create_common_bringup')
    pkg_irobot_create_ignition_bringup = get_package_share_directory('irobot_create_ignition_bringup')
    pkg_turtlebot4_ignition_gui_plugins = get_package_share_directory('turtlebot4_ignition_gui_plugins')
    pkg_irobot_create_description = get_package_share_directory('irobot_create_description')
    pkg_irobot_create_ignition_plugins = get_package_share_directory('irobot_create_ignition_plugins')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    t4_wyman = get_package_share_directory('t4_wyman')
    tb4_ur5e = get_package_share_directory('tb4_ur5e')
    # Paths
    turtlebot4_ros_ign_bridge_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'ros_ign_bridge.launch.py'])
    rviz_launch = PathJoinSubstitution(
        [pkg_turtlebot4_viz, 'launch', 'view_robot.launch.py'])
    turtlebot4_node_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'turtlebot4_nodes.launch.py'])
    create3_nodes_launch = PathJoinSubstitution(
        [pkg_irobot_create_common_bringup, 'launch', 'create3_nodes.launch.py'])
    create3_ignition_nodes_launch = PathJoinSubstitution(
        [pkg_irobot_create_ignition_bringup, 'launch', 'create3_ignition_nodes.launch.py'])
    robot_description_launch = PathJoinSubstitution(
        [pkg_turtlebot4_description, 'launch', 'robot_description.launch.py'])
    dock_description_launch = PathJoinSubstitution(
        [pkg_irobot_create_common_bringup, 'launch', 'dock_description.launch.py'])
    localization_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'localization.launch.py'])
    slam_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'slam.launch.py'])
    nav2_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'nav2.launch.py'])

    for pose_element in ['x', 'y', 'z', 'yaw']:
        ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))

    ign_libgl_path = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE',
        value=['true'])

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_turtlebot4_ignition_bringup, 'worlds'), ':' +
            os.path.join(pkg_irobot_create_ignition_bringup, 'worlds'), ':' +
            os.path.join(t4_wyman,'worlds'), ':' +
            os.path.join(tb4_ur5e,'worlds'), ':' +
            str(Path(pkg_turtlebot4_description).parent.resolve()), ':' +
            str(Path(pkg_irobot_create_description).parent.resolve())])


    ign_gui_plugin_path = SetEnvironmentVariable(
        name='IGN_GUI_PLUGIN_PATH',
        value=[
            os.path.join(pkg_turtlebot4_ignition_gui_plugins, 'lib'), ':' +
            os.path.join(pkg_irobot_create_ignition_plugins, 'lib')])

    # Clock bridge
    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                            '/clock' + '@rosgraph_msgs/msg/Clock' + '[ignition.msgs.Clock'
                        ])

    # Parameters
    param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot4_ignition_bringup, 'config', 'turtlebot4_node.yaml']),
        description='Turtlebot4 Robot param file')

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x0, y0, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    x = OffsetParser(x0, -2)
    y = OffsetParser(y0, 0)
    yaw = LaunchConfiguration('yaw')
    turtlebot4_node_yaml_file = LaunchConfiguration('param_file')

    robot_name = GetNamespacedName(namespace, 'turtlebot4')
    dock_name = GetNamespacedName(namespace, 'standard_dock')

    # Calculate dock offset due to yaw rotation
    dock_offset_x = RotationalOffsetX(0.157, yaw)
    dock_offset_y = RotationalOffsetY(0.157, yaw)
    # Spawn dock at robot position + rotational offset
    x_dock = OffsetParser(x, dock_offset_x)
    y_dock = OffsetParser(y, dock_offset_y)
    # Spawn robot slightly clsoer to the floor to reduce the drop
    # Ensures robot remains properly docked after the drop
    z_robot = OffsetParser(z, -0.0025)
    # Rotate dock towards robot
    yaw_dock = OffsetParser(yaw, 3.1416)
    map_file = os.path.join(t4_wyman,'maps','170map.yaml')

    # Ignition gazebo launch
    ign_gazebo_launch = PathJoinSubstitution(
        [pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'])

    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        launch_arguments=[
            ('ign_args', [LaunchConfiguration('world'),
                          '.sdf',
                          ' -r',
                          ' -v 4',
                          ' --gui-config ',
                          PathJoinSubstitution(
                            [pkg_turtlebot4_ignition_bringup,
                             'gui',
                             LaunchConfiguration('model'),
                             'gui.config'])])
        ]
    )
    # spawn turtlebot
    spawn_robot_group_action = GroupAction([
        PushRosNamespace(namespace),

        # Robot description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_description_launch]),
            launch_arguments=[('model', LaunchConfiguration('model')),
                              ('use_sim_time', LaunchConfiguration('use_sim_time'))]
        ),

        # Dock description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([dock_description_launch]),
            # The robot starts docked
            launch_arguments={'gazebo': 'ignition'}.items(),
        ),

        # Spawn TurtleBot 4
        Node(
            package='ros_ign_gazebo',
            executable='create',
            arguments=['-name', robot_name,
                       '-x', x,
                       '-y', y,
                       '-z', z_robot,
                       '-Y', yaw,
                       '-topic', 'robot_description'],
            output='screen'
        ),



        # ROS IGN bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot4_ros_ign_bridge_launch]),
            launch_arguments=[
                ('model', LaunchConfiguration('model')),
                ('robot_name', robot_name),
                ('dock_name', dock_name),
                ('namespace', namespace),
                ('world', LaunchConfiguration('world'))]
        ),

        # TurtleBot 4 nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot4_node_launch]),
            launch_arguments=[('model', LaunchConfiguration('model')),
                              ('param_file', turtlebot4_node_yaml_file)]
        ),

        # Create 3 nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([create3_nodes_launch]),
            launch_arguments=[
                ('namespace', namespace)
            ]
        ),

        # Create 3 Ignition nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([create3_ignition_nodes_launch]),
            launch_arguments=[
                ('robot_name', robot_name),
                ('dock_name', dock_name),
            ]
        ),

        # RPLIDAR static transforms
        Node(
            name='rplidar_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0', '0.0',
                'rplidar_link', [robot_name, '/rplidar_link/rplidar']],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
        ),

        # OAKD static transform
        # Required for pointcloud. See https://github.com/gazebosim/gz-sensors/issues/239
        Node(
            name='camera_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0',
                '1.5707', '-1.5707', '0',
                'oakd_rgb_camera_optical_frame',
                [robot_name, '/oakd_rgb_camera_frame/rgbd_camera']
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
        ),

    ])
    # SLAM
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_launch]),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time)
        ],
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    # Localization
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot4_navigation, 'launch', 'localization.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'map': map_file
        }.items()
    )

    # Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot4_navigation, 'launch', 'nav2.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time
            # ,
            # 'params_file': PathJoinSubstitution([
            #     t4_wyman, 'config', 'nav2.yaml'
            # ])
        }.items()
    )

    # RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot4_viz, 'launch', 'view_robot.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    



    # ur5e
    x_ur = LaunchConfiguration("x_ur")
    y_ur = LaunchConfiguration("y_ur")
    z_ur = LaunchConfiguration("z_ur")
    use_fake = LaunchConfiguration("use_fake")
    use_gazebo = LaunchConfiguration("use_gazebo")
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
    robotiq_activation_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_activation_controller", "-c", "/controller_manager"],
        output="screen",
    )   

    robotiq_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "-c", "/controller_manager"],
        output="screen",
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
        arguments=[x_ur, y_ur, z_ur, "0", "0", "0", "sensor_base", "target"],
        name="tf_target",
        output="screen"
    )
# sender and rviz
    rviz_ur = Node(
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
    urdf_path = os.path.join(
        get_package_share_directory('ur5e_cartesian_control'),
        'urdf',
        'ur5e_cartesian.urdf'
    )
    spawn_ur5e = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', urdf_path,
            '-entity', 'ur5e',
            '-x', '0', '-y', '0', '-z', '0'
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ign_libgl_path)
    ld.add_action(ign_resource_path)
    ld.add_action(ign_gui_plugin_path)
    ld.add_action(clock_bridge)
    ld.add_action(param_file_cmd)
    ld.add_action(spawn_robot_group_action)

    ld.add_action(localization)
    ld.add_action(slam)
    ld.add_action(nav2)
    ld.add_action(rviz)

    ld.add_action(spawn_ur5e)
    ld.add_action(generate_urdf_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(controller_manager)
    ld.add_action(joint_state_broadcaster)
    ld.add_action(joint_vel_controller)
    ld.add_action(robotiq_activation_controller)
    ld.add_action(robotiq_gripper_controller)
    ld.add_action(cartesian_controller_node)
    ld.add_action(tf_tool)
    ld.add_action(tf_sensor_base)
    ld.add_action(sender_robot_node)
    ld.add_action(sender_target_node)
    # ld.add_action(rviz_ur)

    ld.add_action(ignition_gazebo)
    return ld
