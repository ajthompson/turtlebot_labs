#!/usr/bin/env python

from labs.srv import *
import rospy

# Handle requests to the A* server
def handle_astar(req):
	print "Handling A* Request"

# Initialize the A* server
def astar_server():
	rospy.init_node('astar_server')
	s = rospy.Service('calc_astar', Astar, handle_astar)
	print "Ready to calculate A* Path"
	rospy.spin()
	
if __name__ == "__main__":
	astar_server()