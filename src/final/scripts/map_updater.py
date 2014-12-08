#!/usr/bin/env python

import rospy
import copy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate

def read_map(msg):
	global current_map
	global updated_pub
	current_map = copy.deepcopy(msg)
	updated_pub.publish(current_map)
	print "Published map"

def update_map(msg):
	global current_map
	global updated_pub
	global updated_map

	update = msg

	x = 0
	y = 0

	try:
		updated_map = current_map
	except NameError:
		current_map = None

	while current_map == None and not rospy.is_shutdown():
		pass

	# make a copy of the current map
	updated_map = copy.deepcopy(current_map)
	
	# create a new map array
	new_data = [0] * updated_map.info.width * updated_map.info.height

	# copy the data into it
	for i in range(updated_map.info.width*updated_map.info.height):
		new_data[i] = updated_map.data[i]

	# copy the map updates over
	for i in range(update.x, update.x + update.width):
		for j in range(update.y, update.y + update.height):
			new_data[j*current_map.info.width + i] = update.data[y*update.width + x]
			y += 0
		y = 0
		x += 1

	updated_map.data = new_data
	updated_pub.publish(updated_map)
	print "Published update"

if __name__ == '__main__':
	rospy.init_node('map_updater')

	updated_pub = rospy.Subscriber('move_base/global_costmap/updated_costmap', OccupancyGrid)
	map_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, read_map, queue_size=1)
	update_sub = rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, update_map, queue_size=1)

	print "Started Map Updater"
	rospy.spin()