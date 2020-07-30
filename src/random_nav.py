#! /usr/bin/env python

# Random Nav: A node that sends random spaced out free points as goals to move base in order to have 
# a robot patrol a space randomly.

# Author: Tomi Kalejaiye
# Email: ok93@cornell.edu

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np

class Nav():
  '''Class for bouncing robot around.'''

  def __init__(self):
    ''' Initialization which starts node, and begins subscribing to the map. '''
    self.buffer = 10
    rospy.init_node('random_nav', anonymous=True)
    rospy.loginfo("Initialized Bouncer Node")

    rospy.Subscriber("/map", OccupancyGrid, self.get_goals)

    rospy.on_shutdown(self.stopOnShutdown)

    rospy.spin()


  def check_goal(self,x_start,x_end,y_start,y_end,grid):
    ''' Generate random points, and find out if they are safe '''
    goal = (np.random.randint(x_start+self.buffer,x_end-self.buffer),np.random.randint(y_start+self.buffer,y_end-self.buffer))
    while True:
      goal = (np.random.randint(x_start,x_end),np.random.randint(y_start,y_end))
      safe = (grid[goal[0],goal[1]]==0)  and (grid[goal[0]+self.buffer,goal[1]]==0) and (grid[goal[0],goal[1]+self.buffer]==0) \
        and (grid[goal[0]+self.buffer,goal[1]+self.buffer]==0) and (grid[goal[0]-self.buffer,goal[1]]==0) and (grid[goal[0],goal[1]-self.buffer]==0) \
          and (grid[goal[0]-self.buffer,goal[1]-self.buffer]==0)
      if (safe):
        break
    return goal

  def get_goals(self,msg):
    ''' Get occupancy grid, find safe points in each quadrant, and send them as goals. '''
    height = msg.info.height
    width = msg.info.width
    origin = (msg.info.origin.position.x,msg.info.origin.position.y)
    resolution = msg.info.resolution
    occ_grid = np.array(msg.data).reshape(height,width).T

    while True:
      rospy.loginfo("Sending new goals...")
      q1_goal = self.check_goal(0,width//2,0,height//2,occ_grid)
      q2_goal = self.check_goal(width//2,width,0,height//2,occ_grid)
      q3_goal = self.check_goal(0,width//2,height//2,height,occ_grid)
      q4_goal = self.check_goal(width//2,width,height//2,height,occ_grid)

      goal_list = [q1_goal,q2_goal,q3_goal,q4_goal]

      for goal in goal_list:
        self.movebase_client(goal[0],goal[1],origin,resolution)
      rospy.loginfo("Goals sent.")


  def movebase_client(self,goal_x,goal_y,origin,resolution):
      ''' Action client and communication with move_base '''
      client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
      client.wait_for_server()

      goal = MoveBaseGoal()
      goal.target_pose.header.frame_id = "map"
      goal.target_pose.header.stamp = rospy.Time.now()
      goal.target_pose.pose.position.x = origin[0] + ((goal_x+0.5) * resolution)
      goal.target_pose.pose.position.y = origin[1] + ((goal_y+0.5) * resolution)
      goal.target_pose.pose.orientation.w = 1.0

      client.send_goal(goal)
      wait = client.wait_for_result()
      if not wait:
          rospy.logerr("Action server not available!")
          rospy.signal_shutdown("Action server not available!")
      else:
          return client.get_result()

  def stopOnShutdown(self):
    ''' Shutdown Handler '''

    rospy.loginfo("Random Nav is shutting down...")
    # twist = Twist()
    # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    # twist.linear.x = 0
    # twist.angular.x = 0
    # pub.publish(twist)


if __name__=="__main__":
    Nav()

