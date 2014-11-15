#!/usr/bin/env python

from labs.srv import *
import math
from Queue import PriorityQueue
import rospy, tf
from nav_msgs.msg import Odometry, OccupancyGrid

# create the class for the node
class Node:
	def __init__(self, cost, i_x, i_y, parent):
		self.i_x = i_x
		self.i_y = i_y
		self.cost = cost + self.heuristic(i_x, i_y)
		self.parent = parent

	def heuristic(self):
		return math.sqrt((goal_x - self.i_x)**2 + (goal_y - self.i_y)**2)

# Handle requests to the A* server
def handle_astar(req):
	print "Handling A* Request"

	global start_x
	global start_y
	global goal_x
	global goal_y

	# Get currentlocation
	currentPose = req.startPose
	# get the map
	currentMap = Map
	# get the goal orientation
	goalPose = req.endPose	

	# extract the maps metadata
	resolution = currentMap.info.resolution
	width = currentMap.info.width
	height = currentMap.info.height
	offsetPose = currentMap.info.origin

	# convert the start and end positions to indices in the OccupancyGrid
	start_x = math.floor(currentPose.position.x / resolution)
	start_y = math.floor(currentPose.position.y / resolution)

	frontier = PriorityQueue()

# Initialize the A* server
def astar_server():
	rospy.init_node('astar_server')
	s = rospy.Service('calc_astar', Astar, handle_astar)
	print "Ready to calculate A* Path"
	rospy.spin()

def read_map(msg):
	global Map
	Map = msg
	
if __name__ == "__main__":
	global Map
	global odom_list
	global pub_expanded_cell
	global pub_frontier_cell
	global pub_unexplored_cell 	# not sure if we actually need this?
								# it needlessly complicates things because
								# then we need nodes even if they aren't going
								# to be used.

	map_sub = rospy.Subscriber('/map', OccupancyGrid, read_map, queue_size=1)
	pub_expanded_cell = rospy.Publisher('/astar/expanded', GridCells)
	pub_frontier_cell = rospy.Publisher('/astar/frontier', GridCells)
	pub_unexplored_cell = rospy.Publisher('/astar/unexplored', GridCells)

	odom_list = tf.TransformListener()

	rospy.sleep(rospy.Duration(2, 0))

	astar_server()