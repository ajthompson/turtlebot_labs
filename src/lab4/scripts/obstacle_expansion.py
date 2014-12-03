#!/usr/bin/env python

import math
import rospy
import roslib
import copy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate

# Map topic subscriber callback
def read_map(msg):
	global expanded_pub
	global current_map
	current_map = msg
	expanded_map = expand_obstacles(current_map)
	expanded_pub.publish(expanded_map)
	print "Expanded map published"

def expand_obstacles(map_in):
	expanded = copy.deepcopy(map_in)

	width = expanded.info.width
	height = expanded.info.height
	resolution = expanded.info.resolution

	diameter = 0.230	# radius of the robot in meters

	# calculate by how many cells the obstacles must be expanded
	if (diameter < resolution):
		cells = 0
		print "No expansion necessary"
		return expanded
	else:
		cells = int(math.ceil((diameter / 2) / resolution))
		if cells != 1:
			print "Expansion by %s cells necessary" % cells
		else: 
			print "Expansion by %s cell necessary" % cells

	modified = [0] * width * height
	new_data = [0] * width * height

	for i in range(width*height):
		new_data[i] = expanded.data[i]

	for i in range(width):
		for j in range(height):
			if new_data[j*width + i] >= 100 and not modified[j*width + i]:
				# print "Filling at %s, %s" % (i, j)
				# calculate the size of the cells x cells box around it
				for k in range(i - cells, i + cells + 1):
					for l in range(j - cells, j + cells + 1):
						if k >= 0 and k < width and l >= 0 and l < height:
							# print "\tChecking %s, %s" % (k, l)
							if new_data[l*width + k] < 100 and not modified[l*width + k]:
								# print "\t\tAdding at %s*%s+%s" % (l, width, k)
								new_data[l*width + k] = 100
								modified[l*width + k] = 1

	expanded.data = new_data
	return expanded

def update_map(msg):
	global expanded_pub
	global current_map
	update = msg
	x = 0
	y = 0

	try:
		test = current_map
	except NameError:
		current_map = None

	while current_map== None and not rospy.is_shutdown():
		pass

	new_data = [0] * current_map.info.width * current_map.info.height

	for i in range(current_map.info.width*current_map.info.height):
		new_data[i] = current_map.data[i]

	for i in range(update.x, update.x + update.width):
		for j in range(update.y, update.y + update.height):
			new_data[j*current_map.info.width + i] = update.data[y*update.width + x]
			y += 1
		y = 0
		x += 1

	current_map.data = new_data
	expanded_map = expand_obstacles(current_map)
	expanded_pub.publish(expanded_map)
	print "Expanded update published"

if __name__ == '__main__':
	rospy.init_node('obstacle_expansion')

	global expanded_pub
	global unexpanded
	global current_map

	map_sub = rospy.Subscriber('/map', OccupancyGrid, read_map, queue_size=1) #Callback function to handle mapping original /move_base/global_costmap/costmap
	update_sub = rospy.Subscriber('/map_updates', OccupancyGridUpdate, update_map, queue_size=1) # original /move_base/global_costmap/costmap_updates
	expanded_pub = rospy.Publisher('/expandedmap', OccupancyGrid)

	rospy.spin()
