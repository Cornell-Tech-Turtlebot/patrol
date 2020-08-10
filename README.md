# Patrol

A package that contains nodes for various automated patrol algorithms on a robot running ROS. They have been designed for the Turtlebot3 but can easily be adapted to work with other robots as long as they are have LIDAR scans and are running ROS.

[![Autonomous Patrol Video!](http://img.youtube.com/vi/lCQRPiYSwbE/0.jpg)](https://www.youtube.com/watch?v=lCQRPiYSwbE)

## Nodes

### FixedTurn

![FixedTurn Patrol](https://github.com/Cornell-Tech-Turtlebot/patrol/blob/master/images/fixedturn.gif)

*(20x Speed)*

```fixed_turn``` is a rospy node that implements a deterministic patrol algorithm. The robot will move in a straight line until the LIDAR detects an obstacle in front of it within a certain threshold. Then, the robot will stop and turn clockwise until the space in front of it is free, and then continue moving in a straight line.

### Bounce

![Bounce Patrol](https://github.com/Cornell-Tech-Turtlebot/patrol/blob/master/images/bounce.gif)

*(20x Speed)*

```bounce``` is a rospy node that implements a random sampling patrol algorithm. The robot will move in a straight line until the LIDAR detects an obstacle in front of it within a certain threshold. Then, the robot will stop, a new angle will be randomly sampled, the robot will turn to that angle, and then continue moving in a straight line.


### RandomNav

![RandomNav Patrol](https://github.com/Cornell-Tech-Turtlebot/patrol/blob/master/images/randomnav.gif)

*(20x Speed)*

```random_nav``` is a rospy node that implements a stratified rejection sampling patrol algorithm, but makes use of the move_base and actionlib packages. Given an occupancy grid map of the environment, the algorithm divides the map into 4 quadrant, and randomly samples a cell from each quadrant. If the sampled point is an occupied cell, it is rejected and another point is sampled. When there is a free cell sampled in each quadrant, the cells' coordinates are transformed into the world coordinate frame and sent as nav goals to move_base. Once all 4 goals have been visited, the algorithm repeats.

## How To Run

Clone this repository to the ```src``` directory in your workspace and build.

```fixed_turn``` can be run with a robot that is publishing both a LaserScan message under the topic ```/scan```, and can accept Twist messages under the topic ```/cmd_vel```. 

```bounce``` can be run with a robot that is publishing both a LaserScan message under the topic ```/scan```, an Odometry message under the topic ```/odom``` and can accept Twist messages under the topic ```/cmd_vel```.

```random_nav``` can be run with a robot that is publishing both an OccupancyGrid message under the topic ```/map```, and running ```move_base``` to accept navigation goals. This can be most easily accomplished by running the ```turtlebot3_navigation``` package, which publishes a map and starts ```move_base```. This must be run after the turtlebot has already been brought up, either in the real world or in simulation. The ```turtlebot3_navigation``` package may be launched with the following command ```roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$MAP``` where ```$MAP``` is the map file for the current environment. This map must be generated beforehand using SLAM (support not yet included for dynamic maps).
