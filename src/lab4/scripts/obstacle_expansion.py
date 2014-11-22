#!/usr/bin/env python

import math
import rospy
import roslib
import copy
from nav_msgs.msg import OccupancyGrid

# Map topic subscriber callback
def read_map(msg):
	current_map = msg
	expanded_map = expand_obstacles(current_map)

def expand_obstacles(map_in):
	expanded = copy.deepcopy(map_in)

	global expanded_pub

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

	modified = [0] * (width * height)
	new_data = [0] * (width * height)

	for i in range(width*height):
		new_data[i] = expanded.data[i]

	for i in range(width):
		for j in range(height):
			if new_data[j*width + i] >= 50 and not modified[j*width + i]:
				# occupied, so check for edge cases and expand
				if i > 0 and j > 0:						# set top left cell
					if new_data[(j-1)*width+(i-1)] < 50 and not modified[(j-1)*width+(i-1)]:
						new_data[(j-1)*width+(i-1)] = 100
						modified[(j-1)*width+(i-1)] = 1
				if j > 0:								# set top cell
					if new_data[(j-1)*width+i] < 50 and not modified[(j-1)*width+i]:
						new_data[(j-1)*width+i] = 100
						modified[(j-1)*width+i] = 1
				if i < width - 1 and j > 0:				# set top right cell
					if new_data[(j-1)*width+(i+1)] < 50 and not modified[(j-1)*width+(i+1)]:
						new_data[(j-1)*width+(i+1)] = 100
						modified[(j-1)*width+(i+1)] = 1
				if i < width - 1:						# set right cell
					if new_data[j*width+(i+1)] < 50 and not modified[j*width+(i+1)]:
						new_data[j*width+(i+1)] = 100
						modified[j*width+(i+1)] = 1
				if i < width - 1 and j < height - 1:	# set bottom right cell
					if new_data[(j+1)*width+(i+1)] < 50 and not modified[(j+1)*width+(i+1)]:
						new_data[(j+1)*width+(i+1)] = 100
						modified[(j+1)*width+(i+1)] = 1
				if j < height - 1:						# set bottom cell
					if new_data[(j+1)*width+i] < 50 and not modified[(j+1)*width+i]:
						new_data[(j+1)*width+i] = 100
						modified[(j+1)*width+i] = 1
				if i > 0 and j < height - 1:			# set bottom left cell
					if new_data[(j+1)*width+(i-1)] < 50 and not modified[(j+1)*width+(i-1)]:
						new_data[(j+1)*width+(i-1)] = 100
						modified[(j+1)*width+(i-1)] = 1
				if i > 0:								# set left cell
					if new_data[j*width+(i-1)] < 50 and not modified[j*width+(i-1)]:
						new_data[j*width+(i-1)] = 100
						modified[j*width+(i-1)] = 1

	expanded.data = new_data
	expanded_pub.publish(expanded)
	print "Expanded map published"

if __name__ == '__main__':
	rospy.init_node('obstacle_expansion')

	global expanded_pub

	map_sub = rospy.Subscriber('/map', OccupancyGrid, read_map, queue_size=1) #Callback function to handle mapping
	expanded_pub = rospy.Publisher('/expandedmap', OccupancyGrid)

	rospy.spin()