#!/usr/bin/env python

import rospy
import copy
from nav_msgs.msg import OccupancyGrid, OccupancyGridUpdate

def read_map(msg):
	global current_map
	current_map = copy.deepcopy(msg)

if __name__ == '__main__':
	rospy.init_node('map_updater')

	global current_map

	map_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, read_map, queue_size=1)
	update_sub = rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, update_map, queue_size=1)
	updated_pub = rospy.Subscriber('move_base/global_costmap/updated_costmap')