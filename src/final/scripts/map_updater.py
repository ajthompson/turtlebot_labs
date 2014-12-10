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

	while 1 and not rospy.is_shutdown():
		try:
			test = static_map
			break
		except NameError:
			pass

	new_data = [-1] * current_map.info.width * current_map.info.height

	for i in range(current_map.info.width * current_map.info.height):
		if static_map.data[i] < 0:
			new_data = static_map.data[i]
		else:
			new_data = current_map.data[i]

	current_map.data = new_data
	# updated_pub.publish(current_map)
	print "Published map"

def read_static(msg):
	global static_map
	global updated_pub
	static_map = copy.deepcopy(msg)
	updated_pub.publish(static_map)
	print "Read Static Map"

def update_map(msg):
	global current_map
	global updated_pub
	global updated_map

	update = msg

	x = 0
	y = 0

	have_current = 1
	have_static = 1

	while not have_current and not have_static and not rospy.is_shutdown():
		try:
			updated_map = copy.deepcopy(current_map)
			have_current = 1
		except NameError:
			pass
		try:
			static = copy.deepcopy(static_map)
			have_static = 1
		except NameError:
			pass
	
	# create a new map array
	new_data = [-1] * updated_map.info.width * updated_map.info.height

	# copy the data into it
	for i in range(updated_map.info.width*updated_map.info.height):
		new_data[i] = updated_map.data[i]

	# copy the map updates over
	for i in range(update.x, update.x + update.width):
		for j in range(update.y, update.y + update.height):
			if static.data[j * static.info.width + i] < 0:
				new_data[j*updated_map.info.width + i] = static.data[j * static.info.width + i]
			else:
				new_data[j*updated_map.info.width + i] = update.data[y*update.width + x]
			y += 1
		y = 0
		x += 1

	updated_map.data = new_data
	# updated_pub.publish(updated_map)
	current_map = updated_map
	print "Published update"

def update_static(msg):
	global static_map
	global updated_static
	global updated_pub

	update = msg
	x = 0
	y = 0

	while 1 and not rospy.is_shutdown():
		try:
			updated_static = copy.deepcopy(static_map)
			break
		except NameError:
			pass

	new_data = [-1] * updated_static.info.width * updated_static.info.height

	for i in range(updated_static.info.width * updated_static.info.height):
		new_data[i] = updated_static.data[i]

	for i in range(update.x, update.x + update.width):
		for j in range(update.y, update.y + update.height):
			new_data[j*updated_static.info.width + i] = update.data[y*update.width + x]
			y += 1
		y = 0
		x += 1

	updated_static.data = new_data
	static_map = updated_static
	updated_pub.publish(static_map)
	print "Updated Static Map"

if __name__ == '__main__':
	rospy.init_node('map_updater')

	updated_pub = rospy.Publisher('/move_base/global_costmap/updated_costmap', OccupancyGrid)
	# map_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, read_map, queue_size=1)
	# update_sub = rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, update_map, queue_size=1)
	static_sub = rospy.Subscriber('/map', OccupancyGrid, read_static, queue_size=1)
	static_update_sub = rospy.Subscriber('/map_updates', OccupancyGridUpdate, update_static, queue_size=1)

	print "Started Map Updater"
	rospy.spin()