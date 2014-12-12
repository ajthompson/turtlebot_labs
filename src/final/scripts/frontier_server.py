#!/usr/bin/env python
import rospy, math, copy
import numpy as np
import numpy.ma as ma
from final.srv import *
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, GridCells

# Each cells contains the x and y index in the map
class Cell:
	global DEBUG
	global resolution
	global width
	global height
	global offsetPose
	global data

	DEBUG = 1

	def __init__(self, x, y):
		self.x = x
		self.y = y

	# overload equality method
	def __eq__ (self, other):
		return self.x == other.x and self.y == other.y

	# Create string method to print the cell
	def __str__ (self):
		return "(%s, %s)" % (self.x, self.y)

	def x_to_meters(self):
		return float(self.x * resolution + offsetPose.position.x)

	def y_to_meters(self):
		return float(self.y * resolution + offsetPose.position.y)

# contains the list of frontier ids and the list of cells contained on this frontier
class _Frontier:
	global DEBUG
	global resolution
	global width
	global height
	global offsetPose
	global data

	def __init__(self, f_id, first_cell_x, first_cell_y):
		self.size = 0
		self.f_ids = list()
		self.f_ids.append(f_id)
		self.cells = list()
		if DEBUG:
			self.gridcells = GridCells()
			self.gridcells.header.frame_id = 'map'
			self.gridcells.cell_width = resolution
			self.gridcells.cell_height = resolution
			self.publisher = rospy.Publisher('/frontiers/f%s' % f_id, GridCells)
		self.add_cell(first_cell_x, first_cell_y)
		self.priority = 0

	# overload equality method
	def __eq__ (self, other):
		# select shorter list
		ids_to_check = self.f_ids if len(self.f_ids) < len(other.f_ids) else other.f_ids
		list_to_check = other.f_ids if len(self.f_ids) < len(other.f_ids) else self.f_ids
		# iterate checking if list_to_check contains any ids in ids_to_check
		for i in ids_to_check:
			if i in list_to_check:
				return True
		return False

	# override comparison to sort the list
	def __cmp__ (self, other):
		return other.priority - self.priority

	# override string so we can print the object
	def __str__ (self):
		title = "Frontier of size %s:\n" % self.size

		ids = "\tIDs: "
		for i in self.f_ids:
			ids += "%s " % i
		ids += "\n"

		cells = "\tCells: "
		for j in self.cells:
			cells += "%s " % j
		cells += "\n"
		gridcells = "\tGridCells\n"
		gridcells += str(self.gridcells)
		return title + ids + cells + gridcells

	# adds a cell 
	def add_cell(self, x, y):
		new_cell = Cell(x, y)
		if not new_cell in self.cells:
			self.cells.append(new_cell)
			if DEBUG:	# add gridcell and publish if in debug mode
				self.gridcells.cells.append(Point(x * resolution + offsetPose.position.x + resolution / 2, y * resolution + offsetPose.position.y + resolution / 2, 0))
				# self.publish()
		self.size += 1
		if DEBUG:
			pass
			# print self

	def merge(self, other):
		overlap_counter = 0
		self.f_ids.sort()
		other.f_ids.sort()

		# merge the cells
		for new_cell in other.cells:
			if new_cell not in self.cells:
				self.add_cell(new_cell.x, new_cell.y)
			else:
				overlap_counter += 1

		# add the sizes
		self.size += other.size - overlap_counter

		# when in debug mode, merge the gridcells and publish
		if DEBUG:
			for gridcell in other.gridcells.cells:
				if gridcell not in self.gridcells.cells:
					self.gridcells.cells.append(gridcell)
			# copy over the publisher
			if self.f_ids[0] > other.f_ids[0]:
				dummy_cells = GridCells()
				dummy_cells.header.frame_id = 'map'
				dummy_cells.cell_width = resolution
				dummy_cells.cell_height = resolution
				self.publisher.publish(dummy_cells)
				self.publisher = other.publisher
				self.publish()

		# merge the frontier ids
		for f_id in other.f_ids:
			if f_id not in self.f_ids:
				self.f_ids.append(f_id)
				self.f_ids.sort()

	def set_centroid(self):
		sum_x = 0
		sum_y = 0
		num_cells = 0

		for c in self.cells:
			sum_x += c.x_to_meters()
			sum_y += c.y_to_meters()
			num_cells += 1

		cent_x = sum_x / num_cells
		cent_y = sum_y / num_cells

		newPose = PoseStamped()
		newPose.header.frame_id = 'map'
		newPose.pose.position.x = cent_x + resolution / 2 # add resolution b/c of some
		newPose.pose.position.y = cent_y + resolution / 2 # weird off-by-one error
		newPose.pose.orientation.w = 1
		self.centroid = newPose

	def set_priority(self, pose):
		global current_pose
		current_x = current_pose.pose.position.x
		current_y = current_pose.pose.position.y
		c_x = self.centroid.pose.position.x
		c_y = self.centroid.pose.position.y

		dist = math.sqrt((c_x - current_x)**2 + (c_y - current_y)**2)

		self.priority = 10 * self.size + (1 / dist)
		self.dist = dist

	if DEBUG:
		# publishes the frontier
		def publish(self):
			if len(self.gridcells.cells) == 1:
				if frontiers.count(self) == 1:	
					self.publisher.publish(self.gridcells)
			else:
				self.publisher.publish(self.gridcells)

		def empty(self):
			self.gridcells.cells = list()
			self.publisher.publish(self.gridcells)

def frontier_server():
	rospy.init_node('frontier_server')

	##### USE TO ENABLE DEBUG PUBLISHING OF FRONTIER GRIDCELLS
	global OCCUPIED_THRESHOLD

	# variable assignments
	OCCUPIED_THRESHOLD = 50

	# subscribers
	map_sub = rospy.Subscriber('/move_base/global_costmap/updated_costmap', OccupancyGrid, read_map, queue_size=1) # original /move_base/global_costmap/costmap

	# create service
	s = rospy.Service('calc_frontiers', Frontier, handle_frontiers)

	print "Waiting for request"
	rospy.spin()

def handle_frontiers(req):
	if DEBUG: 
		print "Received request"
	# list of frontiers
	global frontiers
	# the current map
	global current_costmap
	# the current pose
	global current_pose

	current_pose = req.current_pose

	# global map metadata values
	global resolution
	global width
	global height
	global offsetPose
	global data

	# keeps track of the current highest numbered frontier
	global frontier_counter

	# wait for a costmap to be received
	while 1 and not rospy.is_shutdown():
		try:
			current_costmap = copy.deepcopy(costmap)
			break
		except NameError:
			pass
		
	if DEBUG:
		print "Received costmap"

	# extract the metadata
	resolution = current_costmap.info.resolution
	width = current_costmap.info.width
	height = current_costmap.info.height
	offsetPose = current_costmap.info.origin
	old_data = copy.deepcopy(current_costmap.data)

	data = [-1] * width * height

	for i in range(width * height):
		data[i] = old_data[i]

	if DEBUG:
		print "Costmap:"
		print "\tresolution: %s" % resolution
		print "\twidth:      %s" % width
		print "\theight:     %s" % height
		print "\toffsetPose:"
		print offsetPose

	# initialize frontier list
	frontiers = list()
	# initialize frontier counter
	frontier_counter = 2	# frontier values in the map will be from -2...-something, to not confuse
							# them with the positive occupied probability

	# iterate through 
	for i in range(width):
		for j in range(height):
			process_cell(i, j)

	if DEBUG:
		print "Final frontier count: %s" % len(frontiers)

	if DEBUG:
		rospy.sleep(rospy.Duration(0.1))
		publish_frontiers()

	# calculate centroids
	centroids = calc_centroids(frontiers)
	centroid_publishers = list()

	for i in range(len(centroids)):
		if DEBUG:
			print centroids[i]
			centroid_publishers.append(rospy.Publisher('/centroids/c%s' % i, PoseStamped))

	if DEBUG:
		rospy.sleep(rospy.Duration(0.1))
		for i in range(len(centroids)):
			centroid_publishers[i].publish(centroids[i])

	empty_frontiers()

	print "Finished frontier calculation"
	return FrontierResponse(centroids)

# processes a given cell
def process_cell(x, y):
	global frontier_counter
	global frontiers
	global data
	on_frontier = 0
	bordering_frontiers = list()

	# if DEBUG:
		# print "Checking cell [%s, %s] with value %s" % (x, y, data[y * width + x])

	if data[y * width + x] <= -1:
		if x > 0 and y > 0:											# top left
			check = data[(y - 1) * width + (x - 1)]
			if check < -1 and -check not in bordering_frontiers:	# preexisting frontier
				bordering_frontiers.append(-check)
			elif check < OCCUPIED_THRESHOLD and check >= 0:			# on frontier
				on_frontier = 1
		if y > 0:													# top
			check = data[(y - 1) * width + x]
			if check < -1 and -check not in bordering_frontiers:	# preexisting frontier
				bordering_frontiers.append(-check)
			elif check < OCCUPIED_THRESHOLD and check >= 0:			# on frontier
				on_frontier = 1
		if x < width - 1 and y > 0:									# top right
			check = data[(y - 1) * width + (x + 1)]
			if check < -1 and -check not in bordering_frontiers:	# preexisting frontier
				bordering_frontiers.append(-check)
			elif check < OCCUPIED_THRESHOLD and check >= 0:			# on frontier
				on_frontier = 1
		if x < width - 1:											# right
			check = data[y * width + (x + 1)]
			if check < -1 and -check not in bordering_frontiers:	# preexisting frontier
				bordering_frontiers.append(-check)
			elif check < OCCUPIED_THRESHOLD and check >= 0:			# on frontier
				on_frontier = 1
		if x < width - 1 and y < height - 1:						# bottom right
			check = data[(y + 1) * width + (x + 1)]
			if check < -1 and -check not in bordering_frontiers:	# preexisting frontier
				bordering_frontiers.append(-check)
			elif check < OCCUPIED_THRESHOLD and check >= 0:			# on frontier
				on_frontier = 1
		if y < height - 1:											# bottom
			check = data[(y + 1) * width + x]
			if check < -1 and -check not in bordering_frontiers:	# preexisting frontier
				bordering_frontiers.append(-check)
			elif check < OCCUPIED_THRESHOLD and check >= 0:			# on frontier
				on_frontier = 1
		if x > 0 and y < height - 1:								# bottom left
			check = data[(y + 1) * width + (x - 1)]
			if check < -1 and -check not in bordering_frontiers:	# preexisting frontier
				bordering_frontiers.append(-check)
			elif check < OCCUPIED_THRESHOLD and check >= 0:			# on frontier
				on_frontier = 1
		if x > 0:													# left
			check = data[y * width + (x - 1)]
			if check < -1 and -check not in bordering_frontiers:	# preexisting frontier
				bordering_frontiers.append(-check)
			elif check < OCCUPIED_THRESHOLD and check >= 0:			# on frontier
				on_frontier = 1

	if on_frontier:											# on frontier, so fill in
		if len(bordering_frontiers) == 0:					# make new frontier
			data[y * width + x] = -frontier_counter
			new_frontier = _Frontier(frontier_counter, x, y)
			frontiers.append(new_frontier)
			frontier_counter += 1;
		elif len(bordering_frontiers) == 1:					# only one bordering, add to frontier
			data[y * width + x] = -bordering_frontiers.pop(0)
			new_frontier = _Frontier(-data[y * width + x], x, y)
			frontiers[frontiers.index(new_frontier)].merge(new_frontier)
		else:												# multiple bordering, check if merged
			merged_frontier = merge_frontiers(bordering_frontiers)
			data[y * width + x] = -merged_frontier.f_ids[0]
			merged_frontier.add_cell(x, y)
			frontiers.append(merged_frontier)
		publish_frontiers()
		# raw_input("Press ENTER to continue...")

def merge_frontiers(nums):
	global frontiers
	to_merge = list()

	# remove all bordering frontiers from list
	for i in nums:

		dummy = _Frontier(i, 0, 0)

		j = 0
		while j < len(frontiers):
			if dummy == frontiers[j]:
				to_merge.append(frontiers.pop(j))
				j -= 1
			j += 1

	# merge all frontiers into one
	while len(to_merge) > 1:
		to_merge[0].merge(to_merge.pop())

	return to_merge[0]


def get_only_frontier(f_id):
	global frontiers
	i = 0
	for f in frontiers:
		if f_id in f.f_ids:
			return frontiers.pop(i)
		i += 1
	return 

# calculates the centroids of the list of frontiers
def calc_centroids(frontier_list):
	centroids = list()

	for frontier in frontier_list:
		frontier.set_centroid()
		frontier.set_priority(current_pose)

	frontier_list.sort()

	for frontier in frontier_list:
		if frontier.size > 3 and frontier.dist > 0.4:
			centroids.append(frontier.centroid)

	return centroids

def empty_frontiers():
	if DEBUG:
		for f in frontiers:
			f.empty()

def publish_frontiers():
	if DEBUG:
		for f in frontiers:
			f.publish()

# SUBSCRIBER CALLBACKS
def read_map(msg):
	global costmap
	costmap = msg

if __name__ == "__main__":
	
	print "Starting server"
	frontier_server()
	print "Server shutdown"