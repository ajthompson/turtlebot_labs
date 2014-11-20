#!/usr/bin/env python
from lab3.srv import *
import rospy, tf
#from Astar.srv import *
import math
import copy
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

	global start_x
	global start_y
	global goal_x
	global goal_y

	global frontier
	global expanded

	global frontier_grid
	global expanded_grid
	global unexplored_grid

	global current

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

	# create the expanded grid
	expanded_grid = GridCells()
	expanded_grid.header.frame_id = 'map'
	expanded_grid.cell_width = resolution
	expanded_grid.cell_height = resolution

	# create the unexplored grid
	unexplored_grid = GridCells()
	unexplored_grid.header.frame_id = 'map'
	unexplored_grid.cell_width = resolution
	unexplored_grid.cell_height = resolution

	for i in range(width * height):
		frontier_grid.cells.append(Point(10000, 10000, 10000))
		expanded_grid.cells.append(Point(10000, 10000, 10000))
		unexplored_grid.cells.append(Point(10000, 10000, 10000))

	for i in range(width * height, 0):
		frontier_grid.cells.pop(i)
		expanded_grid.cells.pop(i)

	# fill the unexplored grid
	for y in range(height):
		for x in range(width):
			unexplored_grid.cells.insert(y * width + x, Point(x * resolution + offsetPose.position.x, y * resolution + offsetPose.position.y, 0))

	addToFrontier(start_x, start_y)
	print "Creating Priority Queues"
	frontier = PriorityQueue()
	expanded = PriorityQueue()

	# add start node
	print "Creating Start Node"
	current = Node(0, start_x, start_y, None)
	frontier.put((0, current))

	print "Entering while loop"
	while 1 and not rospy.is_shutdown():
		print "Started loop iteration"
		current = frontier.get()[1]

		print "i_x: %s" % current.i_x
		print "i_y: %s" % current.i_y
		print "cost: %s" % current.cost
		print "dist_cost: %s" % current.dist_cost

		if current.i_x == goal_x and current.i_y == goal_y:
			print "Goal node reached at (%s, %s)" % (current.i_x, current.i_y)
			expanded.put(current)
			addToExpanded(current.i_x, current.i_y)
			break

		# add children to frontier
		if current.i_x > 0 and current.i_y > 0:				# above left
			if Map.data[(current.i_y - 1)*width + (current.i_x - 1)] < 50:	# check if occupied		
				print "1"
				child = Node(current.dist_cost, current.i_x - 1, current.i_y - 1, current)
				if (not checkFrontier(child)) and (not checkExpanded(child)):
					print "2"
					if child.i_x == goal_x and child.i_y == goal_y:
						frontier.put((0, child))
						print "3"
					else:
						frontier.put((child.cost, child))
					addToFrontier(child.i_x, child.i_y)
					# print "Add child with cost %s" % child.cost
					print "Added child at %s, %s" % (child.i_x, child.i_y)

		if current.i_y > 0:									# above
			if Map.data[(current.i_y - 1)*width + current.i_x] < 50:		# check if occupied
				print "4"
				child = Node(current.dist_cost, current.i_x, current.i_y - 1, current)
				if (not checkFrontier(child)) and (not checkExpanded(child)):
					if child.i_x == goal_x and child.i_y == goal_y:
						print "5"
						frontier.put((0, child))
					else:
						frontier.put((child.cost, child))
					addToFrontier(child.i_x, child.i_y)
					# print "Add child with cost %s" % child.cost
					print "Added child at %s, %s" % (child.i_x, child.i_y)

		if current.i_x < width and current.i_y > 0:			# above right
			if Map.data[(current.i_y - 1)*width + current.i_x + 1] < 50:	# check if occupied
				print "6"
				child = Node(current.dist_cost, current.i_x + 1, current.i_y - 1, current)
				if (not checkFrontier(child)) and (not checkExpanded(child)):
					if child.i_x == goal_x and child.i_y == goal_y:
						print "7"
						frontier.put((0, child))
					else:
						frontier.put((child.cost, child))
					addToFrontier(child.i_x, child.i_y)
					# print "Add child with cost %s" % child.cost
					print "Added child at %s, %s" % (child.i_x, child.i_y)

		# rospy.sleep(rospy.Duration(2, 0))

		if current.i_x < width:								# right
			if Map.data[current.i_y*width + (current.i_x + 1)] < 50:		# check if occupied
				print "8"
				child = Node(current.dist_cost, current.i_x + 1, current.i_y, current)
				if (not checkFrontier(child)) and (not checkExpanded(child)):
					if child.i_x == goal_x and child.i_y == goal_y:
						print "9"
						frontier.put((0, child))
					else:
						frontier.put((child.cost, child))
					addToFrontier(child.i_x, child.i_y)
					# print "Add child with cost %s" % child.cost
					print "Added child at %s, %s" % (child.i_x, child.i_y)

		if current.i_x < width and current.i_y < height:	# bottom right
			if Map.data[(current.i_y + 1)*width + (current.i_x + 1)] < 50:	# check if occupied
				print "10"
				child = Node(current.dist_cost, current.i_x + 1, current.i_y + 1, current)
				if (not checkFrontier(child)) and (not checkExpanded(child)):
					if child.i_x == goal_x and child.i_y == goal_y:
						print "11"
						frontier.put((0, child))
					else:
						frontier.put((child.cost, child))
					addToFrontier(child.i_x, child.i_y)
					# print "Add child with cost %s" % child.cost
					print "Added child at %s, %s" % (child.i_x, child.i_y)

		if current.i_y < height:							# bottom
			if Map.data[(current.i_y + 1)*width + current.i_x] < 50:		# check if occupied
				child = Node(current.dist_cost, current.i_x, current.i_y + 1, current)	
				print "12"
				if (not checkFrontier(child)) and (not checkExpanded(child)):
					if child.i_x == goal_x and child.i_y == goal_y:
						print "13"
						frontier.put((0, child))
					else:
						frontier.put((child.cost, child))
					addToFrontier(child.i_x, child.i_y)
					# print "Add child with cost %s" % child.cost
					print "Added child at %s, %s" % (child.i_x, child.i_y)

		if current.i_x > 0 and current.i_y < height:		# bottom left
			if Map.data[(current.i_y + 1)*width + (current.i_x - 1)] < 50:	# check if occupied
				print "14"
				child = Node(current.dist_cost, current.i_x - 1, current.i_y + 1, current)
				if (not checkFrontier(child)) and (not checkExpanded(child)):
					if child.i_x == goal_x and child.i_y == goal_y:
						print "15"
						frontier.put((0, child))
					else:
						frontier.put((child.cost, child))
					addToFrontier(child.i_x, child.i_y)
					# print "Add child with cost %s" % child.cost
					print "Added child at %s, %s" % (child.i_x, child.i_y)

		if current.i_x > 0:									# left
			if Map.data[current.i_y*width + (current.i_x - 1)] < 50:		# check if occupied
				print "16"
				child = Node(current.dist_cost, current.i_x - 1, current.i_y, current)
				if (not checkFrontier(child)) and (not checkExpanded(child)):
					if child.i_x == goal_x and child.i_y == goal_y:
						print "17"
						frontier.put((0, child))
					else:
						frontier.put((child.cost, child))
					addToFrontier(child.i_x, child.i_y)
					# print "Add child with cost %s" % child.cost
					print "Added child at %s, %s" % (child.i_x, child.i_y)
		
		# add the current node to the explored nodes
		expanded.put(current)
		addToExpanded(current.i_x, current.i_y)
		print "Finished loop iteration"


	print "Exited loop"
	# create the path object
	path = getPath(goal_x,goal_y,start_x,start_y)

	# fill in the path
	# TODO: fill in path
    #TODO: keep track of direction of robot
	return AstarResponse(path)

# Reconstruct path
# set up array of posestamped objects
#use path documentation

def getPath(goal_x,goal_y,start_x,start_y):
	global path
    #current.i_x = goal_x
    #current.i_y = goal_y

	path = Path()
	path.header.frame_id = 'map'
	
	current_node = current
	
	start_path = PoseStamped()# set up array of posestamped objects
	start_path.pose.position.x= start_x
	start_path.pose.position.y= start_y

	goal_path = PoseStamped()
	goal_path.pose.position.x= goal_x
	goal_path.pose.position.y= goal_y
    # start at the goal and follow the parent chain to the beginning
	path.poses.append(goal_path)

	while goal_path!= start_path and current_node.parent != None and not rospy.is_shutdown():
		parent_path = PoseStamped()
		parent_path.pose.position.x = current_node.parent.i_x
		parent_path.pose.position.y = current_node.parent.i_y
		current_node = current_node.parent
		up = parent_path
		path.poses.append(up)
		goal_path = up

    # reverse the list to give the start-to-goal ordering
	path.poses.reverse()
	#return path
	#while True and not rospy.is_shutdown():
		#path_pub.publish(path)

# Initialize the A* server
def astar_server():
	rospy.init_node('astar_server')

	global path
	global Map

	global pub_expanded_cell
	global pub_frontier_cell
	global pub_unexplored_cell

	s = rospy.Service('calc_astar', Astar, handle_astar)

	map_sub = rospy.Subscriber('/map', OccupancyGrid, read_map, queue_size=1)
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

	# # create the frontier grid to publish
	# frontier_grid = GridCells()
	# frontier_grid.header.frame_id = 'map'
	# frontier_grid.cell_width = resolution
	# frontier_grid.cell_height = resolution

	# # create the expanded grid
	# expanded_grid = GridCells()
	# expanded_grid.header.frame_id = 'map'
	# expanded_grid.cell_width = resolution
	# expanded_grid.cell_height = resolution

	# # create the unexplored grid
	# unexplored_grid = GridCells()
	# unexplored_grid.header.frame_id = 'map'
	# unexplored_grid.cell_width = resolution
	# unexplored_grid.cell_height = resolution

	# # fill the unexplored grid
	# for y in range(height):
	# 	for x in range(width):
	# 		unexplored_grid.cells.append(Point(x * resolution + offsetPose.position.x, y * resolution + offsetPose.position.y, 0))
			
	# 		#pub_frontier_cell.publish(frontier_grid)
	# 		#pub_expanded_cell.publish(expanded_grid)
	# 		#pub_unexplored_cell.publish(unexplored_grid)
	path = Path()
	rospy.Timer(rospy.Duration(0.01), timerCallback)
	print "Ready to calculate A* Path"
	rospy.spin()

def addToFrontier(x, y):
	global frontier_grid
	global unexplored_grid

	index = y * width + x
	# print "Inserting at index %s" % index

	# unexplored_grid.cells.pop(Point(x * resolution + offsetPose.position.x, y * resolution + offsetPose.position.y, 0))
	print "popping - frontier"
	unexplored_grid.cells.pop(index)
	print "inserting - frontier"
	frontier_grid.cells.insert(index, Point(x * resolution + offsetPose.position.x, y * resolution + offsetPose.position.y, 0))
	print "Finished inserting - frontier"

	# pub_unexplored_cell.publish(unexplored_grid)
	# pub_frontier_cell.publish(frontier_grid)

def addToExpanded(x, y):
	global frontier_grid
	global expanded_grid

	index = y * width + x
	# print "Inserting at index %s" % index

	print "popping - expanded"
	frontier_grid.cells.pop(index)
	print "inserting - expanded"
	# frontier_grid.cells.remove(Point(x * resolution + offsetPose.position.x, y * resolution + offsetPose.position.y, 0))
	expanded_grid.cells.insert(index, Point(x * resolution + offsetPose.position.x, y * resolution + offsetPose.position.y, 0))
	print "Finished inserting - expanded"

	# pub_frontier_cell.publish(frontier_grid)
	# pub_expanded_cell.publish(expanded_grid)

def checkFrontier(child):
	print "Entered cehckfrontier"
	if type(frontier_grid.cells[child.i_y * width + child.i_x]) is Point:
		return True
	return False

def checkExpanded(child):
	print "Entered cehck Expanded"
	if type(frontier_grid.cells[child.i_y * width + child.i_x]) is Point:
		return True
	return False

def read_map(msg):
	global Map
	Map = msg

def timerCallback(event):
	try:
		pub_frontier_cell.publish(frontier_grid)
	except NameError:
		pass
	try:
		pub_expanded_cell.publish(expanded_grid)
	except NameError:
		pass
	try:
		pub_unexplored_cell.publish(unexplored_grid)
	except NameError:
		pass
	# path_pub.publish(path)

#waypoints stuff
def calc_waypoints():
	pass
	# find out legth of path and multiply it by the resolution

if __name__ == "__main__":
	global Map
	global odom_list
	global pub_expanded_cell
	global pub_frontier_cell
	global pub_unexplored_cell 	# not sure if we actually need this?
	# it needlessly complicates things because
	# then we need nodes even if they aren't going
	# to be used.

	# path_pub = rospy.Publisher('/TrajectoryPlannerROS/global_plan',Path)
	

	#rospy.sleep(rospy.Duration(2, 0))

	astar_server()

	
