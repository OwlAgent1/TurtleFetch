# TurtleFetch
## People
Stefan Hustrulid

Ruichen Huang

Swastid Jeevan Kasture

## Packages
### turtlefetch_msgs: 
Defines the custom messages and action/services formats used in other packages

### ur5e_actions: 
The server nodes to move the ur5e, open/close gripper, and coordinate pick/place commands

### turtlebot_actions: 
The server nodes to move the ur5e given a control input, (undecided if goal position or velocity commands)
### user_inputs: 
Nodes that coordinate with user inputs and interface with the ur5e_actions and turtlebot_actions nodes, (i.e. the client nodes)

### simulation_stuff: 
Launch gazebo simulation for interfacing with other nodes

## Progress/Timeline/Notes
- Stefan: (3/29/2025) Created a world with 4 walls and a table using boxes. Will likely need to be replaced/improved but should work for now for test spawning the Ur5e and Turtlebot. I will look into definine the msgs and actions formats next.
- Ruichen: (05/08/2025) Simulation part is done with all files uploaded in tb4_simulation. By "ros2 launch tb4_ur5e tb4_ur5e.launch.py", spawning tb4 and ur5e with gripper in the same world. And user can navigate the tb4 to somewhere next to ur5e by Nav2. Ur5e can be controlled by sending stataic transform to pick up the cube.
- Ruichen: (05/08/2025) Done with real UR5e and gripper with files uploaded in rsp_ur5e.
