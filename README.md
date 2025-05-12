# TurtleFetch
## People
Stefan Hustrulid

Ruichen Huang

Swastid Jeevan Kasture

## Description
TurtleFetch is an application of turtlebot4 combined with ur5e with gripper. The turtlebot4 will go to somewhere next to the ur5e using Nav2,and ur5e will do "pick and place" task and place and object on the top of turtlebot4. Then the turtlebot4 will go back to the target position.

## Environment
- Ubuntu 22.04
- ROS2 Humble
- Ignition gazebo Fortress 6.16.0

## Hardware
- Turtlebot4 in Wyman
- UR5e in Wyman
- Robotiq Gripper

## Packages
### RoomMap: 
- Purposes:
Roommap of wyman170
- Functionaility:
Provide a map for turtlebot4 to move around
- Limitations:
Not a real time map.
- contributors:
Stefan Hustrulid and Ruichen Huang

### rsp_ur5e: 
- Purposes:
To Control the real ur5e and gripper
- Functionaility:
Move ur5e end-effortor to the target position and control the status of gripper(open/close)
- Limitations:
Need to send every target postion of ur5e in a terminal
- contributors:
Ruichen Huang

### turtlefetch_turtlebot4: 
- Purposes:
Real Turtlebot Launch file
- Functionaility:
Start the localization, Nav2, map and rviz server of turtlebot4 where it will move following the behavior tree.
- Limitations:
Need to clear up the path for turtlebot4
- contributors:
Stefan Hustrulid

### tb4_simulation: 
- Purposes:
Simulation of all tasks
- Functionaility:
Spawn turtlebot and ur5e in the same world and implement move, wait, pick up and place actions
- Limitations:
Positions are hardcoded
- contributors:
Ruichen Huang

## Progress/Timeline/Notes
- Stefan: (3/29/2025) Created a world with 4 walls and a table using boxes. Will likely need to be replaced/improved but should work for now for test spawning the Ur5e and Turtlebot. I will look into definine the msgs and actions formats next.
- Ruichen: (05/08/2025) Simulation part is done with all files uploaded in tb4_simulation. By "ros2 launch tb4_ur5e tb4_ur5e.launch.py", spawning tb4 and ur5e with gripper in the same world. And user can navigate the tb4 to somewhere next to ur5e by Nav2. Ur5e can be controlled by sending stataic transform to pick up the cube.
- Ruichen: (05/08/2025) Done with real UR5e and gripper with files uploaded in rsp_ur5e.

## Build and Run Instructions
### Build
Clone the main branch and build all packages with <pre>colcon build</pre>
### How to Run
#### rsp_ur5e
cd to rsp_ur5e folder and <pre>source install/setup.bash</pre>
Make sure ur5e is connected and is set to remote control

To start the ur5e and gripper, use <pre>ros2 launch ur5e_cartesian_control ur5e.launch.py use_real:=true use_gripper:=true</pre>
you will see the robot model of ur5e in rviz and the gripper will close and open which indicates the gripper and ur5e is ready for use

To move the ur5e, use <pre>ros2 run tf2_ros static_transform_publisher x y z rx ry rz sensor_base target</pre>
which will send the target to ur5e. Here x, y, z are cartesian positions of ur5e while rx ry rz are the rpy

To control the gripper, use <pre>ros2 action send_goal   /robotiq_gripper_controller/gripper_cmd   control_msgs/action/GripperCommand   "{ command: { position: s, max_effort: 10 } }"</pre>
where s is the status of gripper, it can be any value between 0.0 and 0.8( 0.0 indicates open and 0.8 means close )

#### turtlefetch_turtlebot4
(FILL UP HERE)

#### tb4_simulation
cd to tb4_simulation folder and <pre>source install/setup.bash</pre>
To start, use <pre>ros2 launch tb4_ur5e tb4_ur5e.launch.py</pre>
This will spawn turtlebot and ur5e in the same world. Click 2D Pose Estimation in rviz and set the Nav2 Goal to set the target for turtlebot. When turtlebot arrives at somewhere next to ur5e, send gripper command and position command ( like real ur5e and gripper ) to ur5e to pick up the cube on the ground and place it on turtlebot4

### Videos
(ATTACH A LINK FOR VIDEO)
