#!/usr/bin/env python

import rospy, tf
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import OccupancyGrid, Odometry

if __name__ == '__main__':
	rospy.init_node('lab3_node')
	
	global pub_twist
	global pose
	global odom_tf
	global odom_list