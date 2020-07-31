# Patrol

A package that contains nodes for various automated patrol algorithms on a robot running ROS. They have been designed for the Turtlebot3 but can easily be adapted to work with other robots as long as they are have LIDAR scans and are running ROS.

## Nodes

### FixedTurn

![FixedTurn Patrol](https://github.com/Cornell-Tech-Turtlebot/patrol/blob/master/images/fixedturn.gif)

FixedTurn is a rospy node that implements a deterministic patrol algorithm. The robot will move in a straight line until the LIDAR detects an obstacle in front of it within a certain threshold. Then, the robot will stop and turn clockwise until the space in front of it is free, and then continue moving in a straight line.

### Bounce

![Bounce Patrol](https://github.com/Cornell-Tech-Turtlebot/patrol/blob/master/images/bounce.gif)

Bounce is a rospy node that implements a random sampling patrol algorithm. The robot will move in a straight line until the LIDAR detects an obstacle in front of it within a certain threshold. Then, the robot will stop, a new angle will be randomly sampled, the robot will turn to that angle, and then continue moving in a straight line.


### RandomNav

![RandomNav Patrol](https://github.com/Cornell-Tech-Turtlebot/patrol/blob/master/images/randomnav.gif)

RandomNav is a rospy node that implements a stratified rejection sampling patrol algorithm, but makes use of the move_base and actionlib packages. Given an occupancy grid map of the environment, the algorithm divides the map into 4 quadrant, and randomly samples a cell from each quadrant. If the sampled point is an occupied cell, it is rejected and another point is sampled. When there is a free cell sampled in each quadrant, the cells' coordinates are transformed into the world coordinate frame and sent as nav goals to move_base. Once all 4 goals have been visited, the algorithm repeats.


