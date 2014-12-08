#!/usr/bin/env python
import math
import rospy, tf
import roslib
import copy
from lab4.srv import *
from lab4.msg import *
from lab3.srv import *
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped,Point
import sys, select, termios, tty
from nav_msgs.msg import Odometry,OccupancyGrid,GridCells,Path
from move_base_msgs import MoveBaseActionFeedback
from actionlib_msgs import GoalStatusArray
import random
from lab4 import*
#spin
#frontiers
#astar
#publish message

# bumper spin and recalulate

def calc_frontier_client(frontiers):
	rospy.wait_for_service('calc_frontier')
	try:
		calc_astar = rospy.ServiceProxy('calc_frontier', Frontier)
		resp1 = calc_frontier(frontiers)
		return resp1
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
	except TypeError:
		print "Invalid start or goal position"

# Mapping Callback Function

# This is the program's main function
if __name__ == '__main__':
	# Change this node name to include your username
	rospy.init_node('frontier_client_node')

	
	print "Starting frontier_client"
	
	print "Ending frontier_client!"
