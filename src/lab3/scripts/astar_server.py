#!/usr/bin/env python
from lab3.srv import *
import rospy, tf
#from Astar.srv import *
import math
from Queue import PriorityQueue
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid, Path, GridCells

# create the class for the node
class Node:
	cost = 0
	dist_cost = 0
	i_x = 0
	i_y = 0
	parent = None

	def __init__(self, cost, i_x, i_y, parent):
		self.i_x = i_x
		self.i_y = i_y
		self.cost = cost + self.heuristic()
		self.dist_cost = cost + self.distance()
		self.parent = parent

	def __eq__ (self, other):
		return self.i_x == other.i_x and self.i_y == other.i_y

	def heuristic(self):
		return math.sqrt((goal_x - self.i_x)**2 + (goal_y - self.i_y)**2)

	def distance(self):
		return math.sqrt((self.i_x - self.parent.i_x)**2 + (self.i_y - self.parent.i_y)**2)

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

	# create the base placeholder point
	point = Point()
	point.x = 0
	point.y = 0
	point.z = 0

	# create the frontier grid to publish
	frontier_grid = GridCells()
	frontier_grid.cell_width = resolution
	frontier_grid.cell_height = resolution

	# create the expanded grid
	expanded_grid = GridCells()
	expanded_grid.cell_width = resolution
	expanded_grid.cell_height = resolution

	# create the unexplored grid
	unexplored_grid = GridCells()
	unexplored_grid.cell_width = resolution
	unexplored_grid.cell_height = resolution

	# fill the unexplored grid
	for y in range(height):
		for x in range(width):
			unexplored_grid.cells.append(Point(x * resolution, y * resolution, 0))

	frontier_grid.cells.append(Point(start_x * resolution, start_y * resolution, 0))

	pub_frontier_cell.publish(frontier_grid)
	pub_expanded_cell.publish(expanded_grid)
	pub_unexplored_cell.publish(unexplored_grid)

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
				child = Node(current.dist_cost, current.i_x - 1, current.i_y - 1, current)
				if child not in frontier or child not in explored:
					frontier.put(child)
					addToFrontier(child.i_x, child.i_y)
					print "Added above left to frontier"

		if current.i_y > 0:									# above
			if Map.data[(current.i_y - 1)*width + current.i_x] < 50:		# check if occupied
				child = Node(current.dist_cost, current.i_x, current.i_y - 1, current)
				if child not in frontier or child not in explored:
					frontier.put(child)
					addToFrontier(child.i_x, child.i_y)
					print "Added above to frontier"

		if current.i_x < width and current.i_y > 0:			# above right
			if Map.data[(current.i_y - 1)*width + current.i_x + 1] < 50:	# check if occupied
				child = Node(current.dist_cost, current.i_x + 1, current.i_y - 1, current)
				if child not in frontier or child not in explored:
					frontier.put(child)
					addToFrontier(child.i_x, child.i_y)
					print "Added above right to frontier"

		if current.i_x < width:								# right
			if Map.data[current.i_y*width + (current.i_x + 1)] < 50:		# check if occupied
				child = Node(current.dist_cost, current.i_x + 1, current.i_y, current)
				if child not in frontier or child not in explored:
					frontier.put(child)
					addToFrontier(child.i_x, child.i_y)
					print "Added right to frontier"

		if current.i_x < width and current.i_y < height:	# bottom right
			if Map.data[(current.i_y + 1)*width + (current.i_x + 1)] < 50:	# check if occupied
				child = Node(current.dist_cost, current.i_x + 1, current.i_y + 1, current)
				if child not in frontier or child not in explored:
					frontier.put(child)
					addToFrontier(child.i_x, child.i_y)
					print "Added bottom right to frontier"

		if current.i_y < height:							# bottom
			if Map.data[(current.i_y + 1)*width + current.i_x] < 50:		# check if occupied
				child = Node(current.dist_cost, current.i_x, current.i_y + 1)
				if child not in frontier or child not in explored:
					frontier.put(child)
					addToFrontier(child.i_x, child.i_y)
					print "Added bottom to frontier"

		if current.i_x > 0 and current.i_y < height:		# bottom left
			if Map.data[(current.i_y + 1)*width + (current.i_x - 1)] < 50:	# check if occupied
				child = Node(current.dist_cost, current.i_x - 1, current.i_y + 1)
				if child not in frontier or child not in explored:
					frontier.put(child)
					addToFrontier(child.i_x, child.i_y)
					print "Added bottom left to frontier"

		if current.i_x > 0:									# left
			if Map.data[current.i_y*width + (current.i_x - 1)] < 50:		# check if occupied
				child = Node(current.dist_cost, current.i_x - 1, current.i_y)
				if child not in frontier or child not in explored:
					frontier.put(child)
					addToFrontier(child.i_x, child.i_y)
					print "Added left to frontier"

		# add the current node to the explored nodes
		expanded.put(current)
		addToExpanded(current.i_x, current.i_y)

		# create the path object
		path = Path(goal_x,goal_y,start_x,start_y)

		# fill in the path
		# TODO: fill in path
        #TODO: keep track of direction of robot
		return AstarResponse(path)

# Reconstruct path
def Path(goal_x,goal_y,start_x,start_y):
    current.i_x = copy.copy(goal_x)
    current.i_y = copy.copy(goal_y)
    path = []
    # start at the goal and follow the parent chain to the beginning
    path.append(goal_x)
    path.append(goal_y)
    while current.contents != start.contents:
        up = current.parent
        path.append(up)
        current = up
    # reverse the list to give the start-to-goal ordering
    path.reverse()
    return path

# Initialize the A* server
def astar_server():
	rospy.init_node('astar_server')

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

	rospy.sleep(rospy.Duration(1, 0))

	# get the map
	currentMap = Map

	# extract the maps metadata
	resolution = currentMap.info.resolution
	width = currentMap.info.width
	height = currentMap.info.height
	offsetPose = currentMap.info.origin

	# create the frontier grid to publish
	frontier_grid = GridCells()
	frontier_grid.cell_width = resolution
	frontier_grid.cell_height = resolution

	# create the expanded grid
	expanded_grid = GridCells()
	expanded_grid.cell_width = resolution
	expanded_grid.cell_height = resolution

	# create the unexplored grid
	unexplored_grid = GridCells()
	unexplored_grid.cell_width = resolution
	unexplored_grid.cell_height = resolution

	# fill the unexplored grid
	for y in range(height):
		for x in range(width):
			unexplored_grid.cells.append(Point(x * resolution, y * resolution, 0))

	pub_frontier_cell.publish(frontier_grid)
	pub_expanded_cell.publish(expanded_grid)
	pub_unexplored_cell.publish(unexplored_grid)

	print "Ready to calculate A* Path"
	rospy.spin()

def addToFrontier(x, y):
	global frontier_grid
	global unexplored_grid

	unexplored_grid.remove(Point(x * resolution, y * resolution, 0))
	frontier_grid.append(Point(x * resolution, y * resolution, 0))

	pub_unexplored_cell.publish(unexplored_grid)
	pub_frontier_cell.publish(frontier_grid)

def addToExpanded(x, y):
	global frontier_grid
	global expanded_grid

	frontier_grid.remove(Point(x * resolution, y * resolution, 0))
	expanded_grid.append(Point(x * resolution, y * resolution, 0))

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

	
    #pub_path = rospy.Publisher('/Path',Path)

	

	#rospy.sleep(rospy.Duration(2, 0))

	astar_server()
