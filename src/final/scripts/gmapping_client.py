#!/usr/bin/env python
 
import sys
import rospy
from beginner_tutorials.srv import *
 
def map_client(msg):
	rospy.wait_for_service('static_map')
	try:
			static_map= rospy.ServiceProxy('static_map',GetMap)
			resp1 = map_client(msg)
			return resp1
	except rospy.ServiceException, e:
			print "Service call failed: %s"%e

if __name__ == "__main__":
	while True:

	static_map
