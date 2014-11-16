#!/usr/bin/env python

from labs.srv import *
import math
from Queue import PriorityQueue
import rospy, tf
from nav_msgs.msg import Odometry, OccupancyGrid, Path

# create the class for the node
class Node:
	def __init__(self, cost, i_x, i_y, parent):
		self.i_x = i_x
		self.i_y = i_y
		self.cost = cost + self.heuristic(i_x, i_y)
		self.parent = parent

	def __eq__ (self, other):
		return self.i_x == other.i_x and self.i_y == other.i_y

	def heuristic(self):
		return math.sqrt((goal_x - self.i_x)**2 + (goal_y - self.i_y)**2)

# Handle requests to the A* server
def handle_astar(req):
	print "Handling A* Request"

	global width
	global resolution

	global start_x
	global start_y
	global goal_x
	global goal_y

	global frontier
	global expanded

	global frontier_grid
	global expanded_grid
	global unexplored_grid

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

	goal_x = math.floor(goalPose.position.x / resolution)
	goal_y = math.floor(goalPose.position.y / resolution)

	# create the base point class
	point = Point()
	point.x = 0
	point.y = 0
	point.z = 0

	# create the frontier grid to publish
	frontier_grid = GridCell()
	frontier_grid.cell_width = resolution
	frontier_grid.cell_height = resolution

	# create the expanded grid
	expanded_grid = GridCell()
	expanded_grid.cell_width = resolution
	expanded_grid.cell_height = resolution

	# create the unexplored grid
	unexplored_grid = GridCell()
	unexplored_grid.cell_width = resolution
	unexplored_grid.cell_height = resolution

	# fill the unexplored grid
	for y in range(height):
		for x in range(width):
			point.x = x * resolution
			point.y = y * resolution
			unexplored_grid.cells.append(point)

	frontier = PriorityQueue()

	# add start node
	current = Node(0, start_x, start_y, None)
	frontier.put(Node(0, start_x, start_y, None))

	while 1:
		current = frontier.get()

		if current.i_x == goal_x and current.i_y == goal_y:
			print "Goal node reached at (%s, %s)"
			break

		# add children to frontier
		if current.i_x > 0 and current.i_y > 0:				# above left
			if Map.data[(current.i_y - 1)*width + (current.i_x - 1)] < 50:	# check if occupied
				child = Node(current.cost, current.i_x - 1, current.i_y - 1, current)
				if child not in frontier or child not in explored:
					frontier.put(child)
					addToFrontier(child.i_x, child.i_y)
					print "Added above left to frontier"

		if current.i_y > 0:									# above
			if Map.data[(current.i_y - 1)*width + current.i_x] < 50:		# check if occupied
				child = Node(current.cost, current.i_x, current.i_y - 1, current)
				if child not in frontier or child not in explored:
					frontier.put(child)
					addToFrontier(child.i_x, child.i_y)
					print "Added above to frontier"

		if current.i_x < width and current.i_y > 0:			# above right
			if Map.data[(current.i_y - 1)*width + current.i_x + 1] < 50:	# check if occupied
				child = Node(current.cost, current.i_x + 1, current.i_y - 1, current)
				if child not in frontier or child not in explored:
					frontier.put(child)
					addToFrontier(child.i_x, child.i_y)
					print "Added above right to frontier"

		if current.i_x < width:								# right
			if Map.data[current.i_y*width + (current.i_x + 1)] < 50:		# check if occupied
				child = Node(current.cost, current.i_x + 1, current.i_y, current)
				if child not in frontier or child not in explored:
					frontier.put(child)
					addToFrontier(child.i_x, child.i_y)
					print "Added right to frontier"

		if current.i_x < width and current.i_y < height:	# bottom right
			if Map.data[(current.i_y + 1)*width + (current.i_x + 1)] < 50:	# check if occupied
				child = Node(current.cost, current.i_x + 1, current.i_y + 1, current)
				if child not in frontier or child not in explored:
					frontier.put(child)
					addToFrontier(child.i_x, child.i_y)
					print "Added bottom right to frontier"

		if current.i_y < height:							# bottom
			if Map.data[(current.i_y + 1)*width + current.i_x] < 50:		# check if occupied
				child = Node(current.cost, current.i_x, current.i_y + 1)
				if child not in frontier or child not in explored:
					frontier.put(child)
					addToFrontier(child.i_x, child.i_y)
					print "Added bottom to frontier"

		if current.i_x > 0 and current.i_y < height:		# bottom left
			if Map.data[(current.i_y + 1)*width + (current.i_x - 1)] < 50:	# check if occupied
				child = Node(current.cost, current.i_x - 1, current.i_y + 1)
				if child not in frontier or child not in explored:
					frontier.put(child)
					addToFrontier(child.i_x, child.i_y)
					print "Added bottom left to frontier"

		if current.i_x > 0:									# left
			if Map.data[current.i_y*width + (current.i_x - 1)] < 50:		# check if occupied
				child = Node(current.cost, current.i_x - 1, current.i_y)
				if child not in frontier or child not in explored:
					frontier.put(child)
					addToFrontier(child.i_x, child.i_y)
					print "Added left to frontier"

		# add the current node to the explored nodes
		expanded.put(current)

		# create the path object
		path = Path()

		# fill in the path
		# TODO: fill in path

		return AstarResponse(path)

# Initialize the A* server
def astar_server():
	rospy.init_node('astar_server')
	s = rospy.Service('calc_astar', Astar, handle_astar)
	print "Ready to calculate A* Path"
	rospy.spin()

def addToFrontier(x, y):
	global frontier_grid
	global unexplored_grid

	point = Point()
	point.x = x * resolution
	point.y = y * resolution
	point.z = 0

	unexplored_grid.remove(point)
	frontier_grid.append(point)

	pub_unexplored_cell.publish(unexplored_grid)
	pub_frontier_cell.publish(frontier_grid)

def addToExpanded(x, y):
	global frontier_grid
	global expanded_grid

	point = Point()
	point.x = x * resolution
	point.y = y * resolution
	point.z = 0

	frontier_grid.remove(point)
	expanded_grid.append(point)

	pub_frontier_cell.publish(frontier_grid)
	pub_expanded_cell.publish(expanded_grid)

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