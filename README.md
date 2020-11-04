# BFS-DFS-and-Astar-Path-Planning-Technquies-simulated-on-turtltebot3-Gazebo-ROS


This project is mainly about testing different path planning techniques in a certain world full of obstacles and how turtlebot3 managed to get to the goal position. It tackles 3 path planning technquies which are ( Artificial potential field (APF), Breadth first search (BFS), A*).

First, the control techniques used for the turtlebot3 to move was mainly 2 control techniques which are Lyapunov and Goal to Goal. This package is called Milestone 5 as it is a part of a bigger project. 

Instruction for using this algorithm is just to prepare your workspace and copy this package “Milestone5”, this roslaunch Milestone 5 <<name of the launch file>>.

The launch files available are: -

Turtlebot3_Astar.launch >>> after launching this file you will see the map and movement of the turtltebot3 using A* path planning technique. 

Turtlebot3_Astar_modified.launch >>> the same as A* but with modified algorithm, related to the mechanism of A* itself that it provides more optimized path.

Turtlebot3_BFS.launch >>>  after launching this file you will see the map and movement of the turtltebot3 using A* path planning technique. 

Turtlebot3_ID_world.launch >>>  after launching this file you will see the map and movement of the turtltebot3 using APF path planning technique. 

The related “Python” files associating for all these path planning technquies can be found easily in the src folder. 

If you need any further illustration, do not hesitate to contact me. 

Gasser Elazab
Gasser.elazab@gmail.com



