# TurtleFetch
## People
Stefan Hustrulid

Ruichen Huang

Swastid Jeevan Kasture

## Packages

### tb4_simulation: 
Launch gazebo simulation for interfacing with other nodes
### turtlefetch_msgs: 
Defines the custom messages and action/services formats used in other packages
### rsp_ur5e: 
The server nodes to move the ur5e, open/close gripper, and coordinate pick/place commands
### turtlefetch_turtlebot4: 
Real Turtlebot Launch file

## Progress/Timeline/Notes
- Stefan: (3/29/2025) Created a world with 4 walls and a table using boxes. Will likely need to be replaced/improved but should work for now for test spawning the Ur5e and Turtlebot. I will look into definine the msgs and actions formats next.
- Ruichen: (05/08/2025) Simulation part is done with all files uploaded in tb4_simulation. By "ros2 launch tb4_ur5e tb4_ur5e.launch.py", spawning tb4 and ur5e with gripper in the same world. And user can navigate the tb4 to somewhere next to ur5e by Nav2. Ur5e can be controlled by sending stataic transform to pick up the cube.
- Ruichen: (05/08/2025) Done with real UR5e and gripper with files uploaded in rsp_ur5e.
