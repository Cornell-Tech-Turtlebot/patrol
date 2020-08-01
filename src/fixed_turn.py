#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

import numpy as np

class Bouncer():
  '''Class for bouncing robot around.'''

  def __init__(self):
    self.min_distance = 0.4
    self.too_close = False
    self.range_min = 0

    self.fov = 88

    self.BURGER_MAX_LIN_VEL = 0.22 #max: 0.22
    self.BURGER_MAX_ANG_VEL = 1.84 #max: 2.84

    self.WAFFLE_MAX_LIN_VEL = 0.26
    self.WAFFLE_MAX_ANG_VEL = 1.82

    self.LIN_VEL_STEP_SIZE = 0.01
    self.ANG_VEL_STEP_SIZE = 0.1

    self.control_linear_vel  = 0.0
    self.control_angular_vel  = 0.0
    self.target_linear_vel   = 0.0
    self.target_angular_vel  = 0.0

    self.target_angle = 0

    self.lidar_ranges = []
    self.heading = 0

    self.turtlebot3_model = "burger"

    rospy.init_node('fixedturn')
    print("Initialized Node")

    self.sub = rospy.Subscriber('/scan', LaserScan, self.tooClose)
    self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rospy.on_shutdown(self.stopOnShutdown)

    self.twist = Twist()

  def makeSimpleProfile(self, output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

  def tooClose(self,msg):
    self.range_min = msg.range_min
    self.lidar_ranges = np.array(msg.ranges)
    view = np.array(self.lidar_ranges[0:self.fov/2] + self.lidar_ranges[(359-self.fov/2):359])
    closest_front = np.min(view[self.range_min<view])
    if (closest_front<self.min_distance):
        print("Too Close!")
        self.too_close = True
    else:
      self.too_close = False

  def stopOnShutdown(self):
    ''' Stop robot when shutting down '''

    rospy.loginfo("System is shutting down. Stopping robot...")
    self.twist.linear.x = 0
    self.twist.angular.z = 0
    self.pub.publish(self.twist)
  
  def Turn(self):
    r = rospy.Rate(100)
    while True:
      if (not self.too_close):
        break
      self.target_angular_vel = self.BURGER_MAX_ANG_VEL
      self.target_linear_vel = 0.0

      self.control_linear_vel = self.makeSimpleProfile(self.control_linear_vel, self.target_linear_vel, (self.LIN_VEL_STEP_SIZE/2.0))
      self.twist.linear.x = self.control_linear_vel; self.twist.linear.y = 0.0; self.twist.linear.z = 0.0

      self.control_angular_vel = self.makeSimpleProfile(self.control_angular_vel, self.target_angular_vel, (self.ANG_VEL_STEP_SIZE/2.0))
      self.twist.angular.x = 0.0; self.twist.angular.y = 0.0; self.twist.angular.z = self.control_angular_vel

      self.pub.publish(self.twist)
      r.sleep()

  
  def goStraight(self):
    print("Moving...")
    self.target_angular_vel = 0.0
    self.target_linear_vel = self.BURGER_MAX_LIN_VEL

    self.control_linear_vel = self.makeSimpleProfile(self.control_linear_vel, self.target_linear_vel, (self.LIN_VEL_STEP_SIZE/2.0))
    self.twist.linear.x = self.control_linear_vel; self.twist.linear.y = 0.0; self.twist.linear.z = 0.0

    self.control_angular_vel = self.makeSimpleProfile(self.control_angular_vel, self.target_angular_vel, (self.ANG_VEL_STEP_SIZE/2.0))
    self.twist.angular.x = 0.0; self.twist.angular.y = 0.0; self.twist.angular.z = 0.0

    self.pub.publish(self.twist)


if __name__=="__main__":

    bounce = Bouncer()

    r = rospy.Rate(100) # 100hz
    #try:
    while not rospy.is_shutdown():
      if (bounce.too_close):
        print("Too Close")
        bounce.Turn()

      bounce.goStraight()

      r.sleep()
    #except:
        #print("Error")

    # finally:
    #     twist = Twist()
    #     twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
    #     twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
    #     pub.publish(twist)

  # rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'