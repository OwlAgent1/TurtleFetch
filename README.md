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
