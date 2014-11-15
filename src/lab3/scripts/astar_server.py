#!/usr/bin/env python

from labs.srv import *
import math
import rospy, tf
from nav_msgs.msg import Odometry, OccupancyGrid

# Handle requests to the A* server
def handle_astar(req):
	print "Handling A* Request"

	# Get currentlocation
	currentPose = pose
	# get the map
	currentMap = Map
	# get the goal orientation
	goalPose = req.endPose

	


# Initialize the A* server
def astar_server():
	rospy.init_node('astar_server')
	s = rospy.Service('calc_astar', Astar, handle_astar)
	print "Ready to calculate A* Path"
	rospy.spin()

def read_odometry(msg):
	global pose
	pose = msg.pose.pose

def read_map(msg):
	global Map
	Map = msg
	
if __name__ == "__main__":
	global pose
	global Map
	global odom_list

	pose_sub = rospy.Subscriber('/odom', Odometry, read_odometry, queue_size=1)
	map_sub = rospy.Subscriber('/map', OccupancyGrid, read_map, queue_size=1)

	odom_list = tf.TransformListener()

	rospy.sleep(rospy.Duration(2, 0))

	astar_server()