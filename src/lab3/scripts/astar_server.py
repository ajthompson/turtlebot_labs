#!/usr/bin/env python
from lab3.srv import *
import rospy, tf
#from Astar.srv import *
import math
import copy
import numpy as np
import numpy.ma as ma
from Queue import PriorityQueue
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid, Path, GridCells

# create the class for the node
class Node:
	global nodes
	i_x = 0
	i_y = 0
	cost = 0
	parent = None
	dist_cost = 0

	def __init__(self, cost, i_x, i_y, parent):
		self.i_x = int(i_x)
		self.i_y = int(i_y)
		self.parent = parent
		self.dist_cost = cost + self.distance()
		self.cost = self.dist_cost + self.heuristic()

	def __eq__ (self, other):
		return self.i_x == other.i_x and self.i_y == other.i_y

	def __cmp__ (self, other):
		if (other != None):
			return self.cost - other.cost
		return 9999

	def heuristic(self):
		return math.sqrt((goal_x - self.i_x)**2 + (goal_y - self.i_y)**2)

	def distance(self):
		if self.parent != None:
			return math.sqrt((1.0*self.i_x - 1.0*self.parent.i_x)**2 + (1.0*self.i_y - 1.0*self.parent.i_y)**2)
		return 0

# Handle requests to the A* server
def handle_astar(req):
	print "Handling A* Request"

	global width
	global height
	global resolution
	global offsetPose

	global startPose
	global goalPose

	global start_x
	global start_y
	global goal_x
	global goal_y

	global frontier
	global expanded

	global frontier_grid
	global expanded_grid
	global unexplored_grid

	global frontier_maarray
	global expanded_maarray
	global unexplored_maarray

	global current

	global Map

	# Get currentlocation
	startPose = req.startPose
	currentPose = startPose
	# get the map
	currentMap = Map
	# get the goal orientation
	goalPose = req.endPose

	# extract the maps metadata
	resolution = currentMap.info.resolution
	width = currentMap.info.width
	height = currentMap.info.height
	offsetPose = copy.deepcopy(currentMap.info.origin)
	offsetPose.position.x += resolution / 2
	offsetPose.position.y += resolution / 2

	print "Pose Frame %s" % currentPose.header.frame_id
	print "Resolution: %s" % resolution
	print "Width: %s" % width
	print "Height: %s" % height

	# convert the start and end positions to indices in the OccupancyGrid
	start_x = int(math.floor((currentPose.pose.position.x - offsetPose.position.x + resolution / 2) / resolution))
	start_y = int(math.floor((currentPose.pose.position.y - offsetPose.position.y + resolution / 2) / resolution))

	goal_x = int(math.floor((goalPose.pose.position.x - offsetPose.position.x + resolution / 2) / resolution))
	goal_y = int(math.floor((goalPose.pose.position.y - offsetPose.position.y + resolution / 2) / resolution))

	# Check if start or goal are out of bounds or on impassable space
	if start_x < 0 or start_x >= width or start_y < 0 or start_y >= height:
		try:
			raise TypeError
		except TypeError, e:
			print "Start position out of bounds"
			return AstarResponse(None)
	if goal_x < 0 or goal_x >= width or goal_y < 0 or goal_y >= height:
		try:
			raise TypeError
		except TypeError, e:
			print "Goal Position out of bounds"
			return AstarResponse(None)
	# if currentMap.data[start_x*width + start_y] >= 50:
	# 	try:
	# 		raise TypeError
	# 	except TypeError, e:
	# 		print "Start position on impassable terrain"
	# 		return AstarResponse(None)
	# if currentMap.data[goal_x*width + goal_y] >= 50:
	# 	try:
	# 		raise TypeError
	# 	except TypeError, e:
	# 		print "Goal position on impassable terrain"
	# 		return AstarResponse(None)

	print "Start:"
	print "\tx: %s" % start_x
	print "\ty: %s" % start_y
	print "Goal:"
	print "\tx: %s" % goal_x
	print "\ty: %s" % goal_y

	# create the base placeholder point
	point = Point()
	point.x = 0
	point.y = 0
	point.z = 0

	# create the frontier grid to publish
	frontier_grid = GridCells()
	frontier_grid.header.frame_id = 'map'
	frontier_grid.cell_width = resolution
	frontier_grid.cell_height = resolution
	frontier_maarray = ma.MaskedArray(np.empty(width*height, dtype=object), np.zeros(width*height))

	# create the expanded grid
	expanded_grid = GridCells()
	expanded_grid.header.frame_id = 'map'
	expanded_grid.cell_width = resolution
	expanded_grid.cell_height = resolution
	expanded_maarray = ma.MaskedArray(np.empty(width*height, dtype=object), np.zeros(width*height))

	# create the unexplored grid
	unexplored_grid = GridCells()
	unexplored_grid.header.frame_id = 'map'
	unexplored_grid.cell_width = resolution
	unexplored_grid.cell_height = resolution
	unexplored_maarray = ma.MaskedArray(np.empty(width*height, dtype=object), np.zeros(width*height))

	# fill the unexplored grid
	for y in range(height):
		for x in range(width):
			index = y * width + x
			point = Point(x * resolution + offsetPose.position.x, y * resolution + offsetPose.position.y, 0)
			frontier_maarray.data[index] = point
			expanded_maarray.data[index] = point
			unexplored_maarray.data[index] = point
			frontier_maarray.mask[index] = 1
			expanded_maarray.mask[index] = 1
			if currentMap.data[index] < 50 and currentMap.data[index] != -1:
				unexplored_maarray.mask[index] = 0
			else:
				unexplored_maarray.mask[index] = 1

	addToFrontier(start_x, start_y)

	print "Creating Priority Queues"
	frontier = PriorityQueue()
	expanded = PriorityQueue()

	# add start node
	print "Creating Start Node"
	current = Node(0, start_x, start_y, None)
	frontier.put((0, current))

	while 1 and not rospy.is_shutdown():
		try:
			current = frontier.get()[1]
		except Empty, e:
			print "Goal position is unreachable"
			return AstarResponse(None)

		print "i_x: %s" % current.i_x
		print "i_y: %s" % current.i_y
		print "cost: %s" % current.cost
		print "dist_cost: %s" % current.dist_cost

		if current.i_x == goal_x and current.i_y == goal_y:
			print "Goal node reached at (%s, %s)" % (current.i_x, current.i_y)
			expanded.put(current)
			addToExpanded(current.i_x, current.i_y)
			rospy.sleep(rospy.Duration(0.5))
			break

		# add children to frontier
		if current.i_x > 0 and current.i_y > 0:				# above left
			if currentMap.data[(current.i_y - 1)*width + (current.i_x - 1)] < 50:	# check if occupied
				child = Node(current.dist_cost, current.i_x - 1, current.i_y - 1, current)
				if checkFrontier(child) and checkExpanded(child):
					if child.i_x == goal_x and child.i_y == goal_y:
						frontier.put((0, child))
					else:
						frontier.put((child.cost, child))
					addToFrontier(child.i_x, child.i_y)

		if current.i_y > 0:									# above
			if currentMap.data[(current.i_y - 1)*width + current.i_x] < 50:		# check if occupied
				child = Node(current.dist_cost, current.i_x, current.i_y - 1, current)
				if checkFrontier(child) and checkExpanded(child):
					if child.i_x == goal_x and child.i_y == goal_y:
						frontier.put((0, child))
					else:
						frontier.put((child.cost, child))
					addToFrontier(child.i_x, child.i_y)

		if current.i_x < width - 1 and current.i_y > 0:			# above right
			if currentMap.data[(current.i_y - 1)*width + current.i_x + 1] < 50:	# check if occupied
				child = Node(current.dist_cost, current.i_x + 1, current.i_y - 1, current)
				if checkFrontier(child) and checkExpanded(child):
					if child.i_x == goal_x and child.i_y == goal_y:
						frontier.put((0, child))
					else:
						frontier.put((child.cost, child))
					addToFrontier(child.i_x, child.i_y)

		if current.i_x < width - 1:								# right
			if currentMap.data[current.i_y*width + (current.i_x + 1)] < 50:		# check if occupied
				child = Node(current.dist_cost, current.i_x + 1, current.i_y, current)
				if checkFrontier(child) and checkExpanded(child):
					if child.i_x == goal_x and child.i_y == goal_y:
						frontier.put((0, child))
					else:
						frontier.put((child.cost, child))
					addToFrontier(child.i_x, child.i_y)

		if current.i_x < width - 1 and current.i_y < height - 1:	# bottom right
			if currentMap.data[(current.i_y + 1)*width + (current.i_x + 1)] < 50:	# check if occupied
				child = Node(current.dist_cost, current.i_x + 1, current.i_y + 1, current)
				if checkFrontier(child) and checkExpanded(child):
					if child.i_x == goal_x and child.i_y == goal_y:
						frontier.put((0, child))
					else:
						frontier.put((child.cost, child))
					addToFrontier(child.i_x, child.i_y)

		if current.i_y < height - 1:							# bottom
			if currentMap.data[(current.i_y + 1)*width + current.i_x] < 50:		# check if occupied
				child = Node(current.dist_cost, current.i_x, current.i_y + 1, current)
				if checkFrontier(child) and checkExpanded(child):
					if child.i_x == goal_x and child.i_y == goal_y:
						frontier.put((0, child))
					else:
						frontier.put((child.cost, child))
					addToFrontier(child.i_x, child.i_y)

		if current.i_x > 0 and current.i_y < height - 1:		# bottom left
			if currentMap.data[(current.i_y + 1)*width + (current.i_x - 1)] < 50:	# check if occupied
				child = Node(current.dist_cost, current.i_x - 1, current.i_y + 1, current)
				if checkFrontier(child) and checkExpanded(child):
					if child.i_x == goal_x and child.i_y == goal_y:
						frontier.put((0, child))
					else:
						frontier.put((child.cost, child))
					addToFrontier(child.i_x, child.i_y)
	
		if current.i_x > 0:									# left
			if currentMap.data[current.i_y*width + (current.i_x - 1)] < 50:		# check if occupied
				child = Node(current.dist_cost, current.i_x - 1, current.i_y, current)
				if checkFrontier(child) and checkExpanded(child):
					if child.i_x == goal_x and child.i_y == goal_y:
						frontier.put((0, child))
					else:
						frontier.put((child.cost, child))
					addToFrontier(child.i_x, child.i_y)

					# print "Add child with cost %s" % child.cost
					print "Added child at %s, %s" % (child.i_x, child.i_y)
		
		# add the current node to the explored nodes
		expanded.put(current)
		addToExpanded(current.i_x, current.i_y)

	publishStuff()
	# create the path object
	path = getPath(goal_x,goal_y,start_x,start_y)
	rospy.sleep(rospy.Duration(0.5))
	publishStuff()

	# fill in the path
	# TODO: fill in path
    #TODO: keep track of direction of robot
	return AstarResponse(path)

# Reconstruct path
# set up array of posestamped objects
#use path documentation

def getPath(goal_x,goal_y,start_x,start_y):
	print "Generating Path"
	global path_new

	path_new = Path()
	path_new.header.frame_id = 'map'
	
	current_node = current

	# 1 2 3
	# 8 0 4
	# 7 6 5
	direction = 0
	
	# start_path_new = PoseStamped()# set up array of posestamped objects
	# start_path_new.pose.position.x= start_x
	# start_path_new.pose.position.y= start_y

	# goal_path_new = PoseStamped()
	# goal_path_new.pose.position.x= goal_x
	# goal_path_new.pose.position.y= goal_y
 #    # start at the goal and follow the parent chain to the beginning
	# path_new.poses.append(goal_path_new)

	# while goal_path_new!= start_path_new and current_node.parent != None and not rospy.is_shutdown():
	# 	parent_path_new = PoseStamped()
	# 	parent_path_new.pose.position.x = current_node.parent.i_x
	# 	parent_path_new.pose.position.y = current_node.parent.i_y
	# 	current_node = current_node.parent
	# 	up = parent_path_new
	# 	path_new.poses.append(up)
	# 	goal_path_new = up

	pose = PoseStamped()
	pose.header.frame_id = 'map'
	pose.pose.position.x = current_node.i_x * resolution + offsetPose.position.x
	pose.pose.position.y = current_node.i_y * resolution + offsetPose.position.y
	pose.pose.orientation = goalPose.pose.orientation
	path_new.poses.append(pose)

	while current_node.parent != None:
		print "Checking node at %s, %s" % (current_node.i_x, current_node.i_y)
		diff_x = current_node.i_x - current_node.parent.i_x
		diff_y = current_node.i_y - current_node.parent.i_y

		if diff_x == 1 and diff_y == 1:
			new_dir = 1
		elif diff_x == 0 and diff_y == 1:
			new_dir = 2
		elif diff_x == -1 and diff_y == 1:
			new_dir = 3
		elif diff_x == -1 and diff_y == 0:
			new_dir = 4
		elif diff_x == -1 and diff_y == -1:
			new_dir = 5
		elif diff_x == 0 and diff_y == -1:
			new_dir = 6
		elif diff_x == 1 and diff_y == -1:
			new_dir = 7
		elif diff_x == 1 and diff_y == 0:
			new_dir = 8

		if new_dir != direction:
			print "Creating waypoint at %s, %s" % (current_node.i_x, current_node.i_y)
			pose = copy.deepcopy(pose)
			pose.pose.position.x = current_node.i_x * resolution + offsetPose.position.x
			pose.pose.position.y = current_node.i_y * resolution + offsetPose.position.y
			pose.pose.orientation.w = 1

			if direction == 1:
				pose.pose.orientation.z = math.cos(3 * math.pi / 4) / 2
			elif direction == 2:
				pose.pose.orientation.z = math.cos(math.pi / 2) / 2
			elif direction == 3:
				pose.pose.orientation.z = math.cos(math.pi / 4) / 2
			elif direction == 4:
				pose.pose.orientation.z = math.cos(0) / 2
			elif direction == 5:
				pose.pose.orientation.z = math.cos(7 * math.pi / 4) / 2
			elif direction == 6:
				pose.pose.orientation.z = math.cos(3 * math.pi / 2) / 2
			elif direction == 7:
				pose.pose.orientation.z = math.cos(5 * math.pi / 4) / 2
			elif direction == 8:
				pose.pose.orientation.z = math.cos(math.pi) / 2

			direction = new_dir
			path_new.poses.append(pose)

		current_node = current_node.parent

	pose = copy.deepcopy(pose)
	pose.pose.position.x = current_node.i_x * resolution + offsetPose.position.x
	pose.pose.position.y = current_node.i_y * resolution + offsetPose.position.y
	pose.pose.orientation = startPose.pose.orientation

	path_new.poses.append(pose)
    # reverse the list to give the start-to-goal ordering
	path_new.poses.reverse()
	return path_new

# Initialize the A* server
def astar_server():
	rospy.init_node('astar_server')

	global path
	global Map

	global pub_expanded_cell
	global pub_frontier_cell
	global pub_unexplored_cell

	s = rospy.Service('calc_astar', Astar, handle_astar)

	map_sub = rospy.Subscriber('/expandedmap', OccupancyGrid, read_map, queue_size=1)
	pub_expanded_cell = rospy.Publisher('/astar/expanded', GridCells)
	pub_frontier_cell = rospy.Publisher('/astar/frontier', GridCells)
	pub_unexplored_cell = rospy.Publisher('/astar/unexplored', GridCells)

	global width
	global height
	global resolution
	global offsetPose

	global frontier_grid
	global expanded_grid
	global unexplored_grid

	try:
		test = Map
	except NameError:
		Map = None

	while Map == None and not rospy.is_shutdown():
		pass

	# get the map
	currentMap = Map

	# extract the maps metadata
	resolution = currentMap.info.resolution
	width = currentMap.info.width
	height = currentMap.info.height
	offsetPose = currentMap.info.origin

	path = Path()
	print "Ready to calculate A* Path"
	rospy.spin()

def addToFrontier(x, y):
	global frontier_maarray
	global unexplored_maarray
	index = y * width + x
	unexplored_maarray.mask[index] = 1
	frontier_maarray.mask[index] = 0
	publishStuff()

def addToExpanded(x, y):
	global frontier_maarray
	global expanded_maarray
	index = y * width + x
	frontier_maarray.mask[index] = 1
	expanded_maarray.mask[index] = 0
	publishStuff()

def checkFrontier(child):
	return frontier_maarray.mask[child.i_y * width + child.i_x]


def checkExpanded(child):
	return expanded_maarray.mask[child.i_y * width + child.i_x]

def read_map(msg):
	global Map
	Map = msg

def publishStuff():

	try:
		frontier_grid.cells = frontier_maarray[~frontier_maarray.mask].data.tolist()
		pub_frontier_cell.publish(frontier_grid)
	except NameError:
		pass
	try:
		expanded_grid.cells = expanded_maarray[~expanded_maarray.mask].data.tolist()
		pub_expanded_cell.publish(expanded_grid)
	except NameError:
		pass
	# try:
	# 	unexplored_grid.cells = unexplored_maarray[~unexplored_maarray.mask].data.tolist()
	# 	pub_unexplored_cell.publish(unexplored_grid)
	# except NameError:

if __name__ == "__main__":
	global Map
	global odom_list
	global pub_expanded_cell
	global pub_frontier_cell
	global pub_unexplored_cell

	astar_server()
