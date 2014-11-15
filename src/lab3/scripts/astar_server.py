#!/usr/bin/env python

from labs.srv import *
import math
import rospy, tf
from nav_msgs.msg import Odometry, OccupancyGrid

# Handle requests to the A* server
def handle_astar(req):
	print "Handling A* Request"

	# Get currentlocation
	currentPose = req.startPose
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

# Generates the heuristic for the position
def heuristic(pose, destPose):
	return math.sqrt((pose.position.x - destPose.position.x)**2 + (pose.position.y - destPose.position.y)**2)

def read_map(msg):
	global Map
	Map = msg
	
if __name__ == "__main__":
	global Map
	global odom_list
	global pub_expanded_cell
	global pub_frontier_cell
	global pub_unexplored_cell

	map_sub = rospy.Subscriber('/map', OccupancyGrid, read_map, queue_size=1)
	pub_expanded_cell = rospy.Publisher('/astar/expanded', GridCells)
	pub_frontier_cell = rospy.Publisher('/astar/frontier', GridCells)
	pub_unexplored_cell = rospy.Publisher('/astar/unexplored', GridCells)

	odom_list = tf.TransformListener()

	rospy.sleep(rospy.Duration(2, 0))

	astar_server()