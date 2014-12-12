#!/usr/bin/env python

import rospy
import copy
import math
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import PoseStamped

class Cell:

	def __init__(self, x, y, value):
		self.x = x
		self.y = y
		self.value = value

class CheckLine:
	global checked_cells
	global new_map
	global new_map_data
	global OBSTACLE_THRESHOLD

	def __init__ (self, start_i, start_j, robotPose):
		self.pose = robotPose
		self.start_i = start_i
		self.start_j = start_j
		self.start_x = self.index_to_meters(start_i, 'x')
		self.start_y = self.index_to_meters(start_j, 'y')
		self.goal_x = robotPose.pose.position.x
		self.goal_y = robotPose.pose.position.y
		self.goal_i = self.meters_to_index(self.goal_x, 'x')
		self.goal_j = self.meters_to_index(self.goal_y, 'y')
		self.angle = self.calc_angle(self.start_x, self.start_y, self.goal_x, self.goal_y)
		self.increment = self.calc_overall_increment(self.angle)
		self.inc_x = self.calc_increment(self.increment, self.angle, 'x')
		self.inc_y = self.calc_increment(self.increment, self.angle, 'y')

	def index_to_meters(self, index, axis):
		if axis == 'x':		# compute x value
			return float(index * resolution + offsetPose.position.x)
		elif axis == 'y':	# compute y value
			return float(index * resolution + offsetPose.position.y)

	def meters_to_index(self, meters, axis):
		if axis == 'x':		# compute x value
			return int(math.floor(meters - offsetPose.position.x) / resolution)
		elif axis == 'y':	# compute y value
			return int(math.floor(meters - offsetPose.position.y) / resolution)

	def calc_angle(self, x1, y1, x2, y2):
		return math.atan2(y2 - y1, x2 - x1)

	def calc_increment(self, inc, angle, axis):
		if axis == 'x':
			return inc * math.cos(angle)
		elif axis == 'y':
			return inc * math.sin(angle)

	# calculate the overall increment (straight line) by finding the distance across 1/8
	# of a cell at the given angle. If the line crosses closer to the edge 
	def calc_overall_increment(self, angle):
		return abs(1 / (32 * math.cos(angle)))

	def check_path(self):
		global checked_cells
		global new_map
		global new_map_data
		i = 0
		found_obstacle = 0	

		new_x = int(math.floor(i * self.inc_x + self.start_i))
		new_y = int(math.floor(i * self.inc_y + self.start_j))
		index = new_y * new_map.info.width + new_x

		if start_x > self.goal_i and start_y > self.goal_j:
			end_case = 0
		elif start_x > self.goal_i and start_y <= self.goal_j:
			end_case = 1
		elif start_x <= self.goal_i and start_y > self.goal_j:
			end_case = 2
		elif start_x <= self.goal_i and start_y <= self.goal_j:
			end_case = 3
	
		while 1 and not rospy.is_shutdown() and new_x >= 0 and new_x < new_map.info.width and new_y >= 0 and new_y < new_map.info.height:

			if not checked_cells[index] or found_obstacle:
				checked_cells[index] = 1
				if new_map.data[index] >= OBSTACLE_THRESHOLD:
					new_map_data[index] = 100
					found_obstacle = 1
				elif new_map.data[index] < OBSTACLE_THRESHOLD and found_obstacle:
					new_map_data[index] = 0
			if end_case == 0:
				if new_x <= self.goal_i and new_y <= self.goal_j:
					break
			elif end_case == 1:
				if new_x <= self.goal_i and new_y > self.goal_j:
					break
			elif end_case == 2:
				if new_x > self.goal_i and new_y <= self.goal_j:
			elif end_case == 3:
				if new_x > self.goal_i and new_y > self.goal_j:

			new_x = int(math.floor(i * self.inc_x + self.start_i))
			new_y = int(math.floor(i * self.inc_y + self.start_j))
			index = new_y * new_map.info.width + new_x
			i += 1

def read_map(msg):
	global current_map
	global resolution
	global width
	global height
	global offsetPose

	current_map = msg
	resolution = current_map.info.resolution
	width = current_map.info.width
	height = current_map.info.height
	offsetPose = current_map.info.origin
	print "OffsetPose:"
	print offsetPose

	if not wait:
		create_static_map(current_map, current_pose)

def update_map(msg):
	global current_map
	update = msg
	x = 0
	y = 0

	have_current = 0

	# Wait for map to be updated
	while not have_current and not rospy.is_shutdown():
		try:
			updated_map = copy.deepcopy(current_map)
			have_current = 1
		except NameError:
			pass

	# create a new data array
	new_data = [0] * updated_map.info.width * updated_map.info.height

	# copy the data into it
	for i in range(updated_map.info.width):
		for j in range(updated_map.info.height):
			if i in range(update.x, update.x + update.width) and j in range(update.y, update.y + update.height):
				new_data[j * updated_map.info.width + i] = update.data[(j - update.y) * update.width + (i - update.x)]
			else:
				new_data[j * updated_map.info.width + i] = updated_map.data[j * updated_map.info.width + i]

	updated_map.data = new_data
	current_map = updated_map
	if not wait:
		create_static_map(current_map, current_pose)

def get_pose(msg):
	global current_pose
	current_pose = msg

def create_static_map(current, c_pose):
	global static_pub
	global checked_cells
	global new_map
	global new_map_data
	global wait

	wait = 1

	checked_cells = [0] * current.info.width * current.info.height

	new_map = copy.deepcopy(current)
	new_map_data = [-1] * new_map.info.width * new_map.info.height

	pose = copy.deepcopy(c_pose)

	lines = list()

	# traverse the edge indices to add all lines from edge to current pose
	for i in range(new_map.info.width):					# top left to top right
		lines.append(CheckLine(i, 0, pose))
	for j in range(1, new_map.info.height):				# top right to bottom right
		lines.append(CheckLine(new_map.info.width - 1, j, pose))
	for i in range(new_map.info.width - 1):				# bottom left to bottom right
		lines.append(CheckLine(i, new_map.info.height - 1, pose))
	for j in range(1, new_map.info.height - 1):			# top left to bottom left
		lines.append(CheckLine(0, j, pose))

	count = 0
	for l in lines:
		l.check_path()
		new_map.data = new_map_data
		static_pub.publish(new_map)
		count += 1

	new_map.data = new_map_data
	print "Static map published"
	static_pub.publish(new_map)
	wait = 0

if __name__ == '__main__':
	global wait
	global static_pub
	global OBSTACLE_THRESHOLD
	OBSTACLE_THRESHOLD = 100
	wait = 0
	rospy.init_node('static_builder')

	static_pub = rospy.Publisher('/static_map', OccupancyGrid)
	map_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, read_map, queue_size=1)
	update_sub = rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, update_map, queue_size=1)
	pose_sub = rospy.Subscriber('/poseconv', PoseStamped, get_pose, queue_size=1)

	print "Started static map builder"
	rospy.spin()