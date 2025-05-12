# TurtleFetch
## People
Stefan Hustrulid

Ruichen Huang

Swastid Jeevan Kasture

## Description
TurtleFetch is an application of turtlebot4 combined with ur5e with gripper. The turtlebot4 will go to somewhere next to the ur5e using Nav2,and ur5e will do "pick and place" task and place and object on the top of turtlebot4. Then the turtlebot4 will go back to the target position.

## Packages
### RoomMap: 
##Purposes:
Roommap of wyman170
##Functionaility:
Provide a map for turtlebot4 to move around
##Limitations:
Not a real time map.
##contributors:
Stefan Hustrulid and Ruichen Huang

### rsp_ur5e: 
##Purposes:
To Control the real ur5e and gripper
##Functionaility:
Move ur5e end-effortor to the target position and control the status of gripper(open/close)
##Limitations:
Need to send every target postion of ur5e in a terminal
##contributors:
Ruichen Huang

### turtlefetch_turtlebot4: 
##Purposes:
Real Turtlebot Launch file
##Functionaility:
Start the localization, Nav2, map and rviz server of turtlebot4 where it will move following the behavior tree.
##Limitations:
Need to clear up the path for turtlebot4
##contributors:
Stefan Hustrulid

### tb4_simulation: 
#Purposes:
Simulation of all tasks
#Functionaility:
Spawn turtlebot and ur5e in the same world and implement move, wait, pick up and place actions
#Limitations:
Positions are hardcoded
#contributors:
Ruichen Huang



## Progress/Timeline/Notes
- Stefan: (3/29/2025) Created a world with 4 walls and a table using boxes. Will likely need to be replaced/improved but should work for now for test spawning the Ur5e and Turtlebot. I will look into definine the msgs and actions formats next.
- Ruichen: (05/08/2025) Simulation part is done with all files uploaded in tb4_simulation. By "ros2 launch tb4_ur5e tb4_ur5e.launch.py", spawning tb4 and ur5e with gripper in the same world. And user can navigate the tb4 to somewhere next to ur5e by Nav2. Ur5e can be controlled by sending stataic transform to pick up the cube.
- Ruichen: (05/08/2025) Done with real UR5e and gripper with files uploaded in rsp_ur5e.
