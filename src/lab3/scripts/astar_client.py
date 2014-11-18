#!/usr/bin/env python

import math
import rospy, tf
import roslib
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped,Point
import sys, select, termios, tty
from nav_msgs.msg import Odometry,OccupancyGrid,GridCells
import random

#from lab3.srv import *

# Add additional imports for each of the message types used

# Add additional imports for each of the message types used
def make_obstacles():
    pass
    global point
    global Map
    global counter
    i=0
    counter = 0
    Cell = GridCells()
    point = Point()
    point2 = Point()
    point3 = Point()
    point4 = Point()
    point5 = Point()
    gridCell_pub.publish(Cell)
    Cell.cell_width = .5
    Cell.cell_height = .5
    Cell.header.frame_id = 'map'
    point.x = random.uniform(Map.origin.position.x,Map.width* Map.resolution+Map.origin.position.x-Cell.cell_width) 
    point.y = random.uniform(Map.origin.position.y,Map.height* Map.resolution+Map.origin.position.y-Cell.cell_height) 
    point.z = 0
    Cell.cells.append(point)

    point2.x = random.uniform(Map.origin.position.x,Map.width* Map.resolution+Map.origin.position.x-Cell.cell_width) 
    point2.y = random.uniform(Map.origin.position.y,Map.height* Map.resolution+Map.origin.position.y-Cell.cell_height) 
    point2.z = 0
    Cell.cells.append(point2)

    point3.x = random.uniform(Map.origin.position.x,Map.width* Map.resolution+Map.origin.position.x-Cell.cell_width) 
    point3.y = random.uniform(Map.origin.position.y,Map.height* Map.resolution+Map.origin.position.y-Cell.cell_height) 
    point3.z = 0
    Cell.cells.append(point3)

    point4.x = random.uniform(Map.origin.position.x,Map.width* Map.resolution+Map.origin.position.x-Cell.cell_width) 
    point4.y = random.uniform(Map.origin.position.y,Map.height* Map.resolution+Map.origin.position.y-Cell.cell_height) 
    point4.z = 0
    Cell.cells.append(point4)

    point5.x = random.uniform(Map.origin.position.x,Map.width* Map.resolution+Map.origin.position.x-Cell.cell_width) 
    point5.y = random.uniform(Map.origin.position.y,Map.height* Map.resolution+Map.origin.position.y-Cell.cell_height) 
    point5.z = 0
    Cell.cells.append(point5)
    
    while True and  not rospy.is_shutdown():
        gridCell_pub.publish(Cell)
    
       


#Service Proxy?

def calc_astar_client(start_pose, goal_pose):
	pass
	rospy.wait_for_service('calc_astar')
	try:
		calc_astar = rospy.ServiceProxy('calc_astar', Astar)
		resp1 = calc_astar(start_pose,goal_pose)
		return resp1
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


#Odometry Callback function.
def read_odometry(msg):
   global pose 	
   global theta
   global cpose
   pose = msg.pose.pose
   cpose = msg.pose.pose.orientation.z
   theta = math.asin(cpose)*2

 
#Bumper Event Callback function
def readBumper(msg):
    global buttonPress
    if (msg.state == 1):
        # What should happen when the bumper is pressed?
        buttonPress =1

# Mapping Callback Function
def readMap(msg):

    global Map

    Map = msg.info
    
    print Map.height
    print Map.width
    print Map.resolution

#Initial Position Callback Function to send start points to robot
def readiPose(msg):
    global initialPose
    
    initialpose = msg.PoseWithCovariance.pose
    
    print initialpose
pass

#Move Base Simple Callback Function to send endpoints to robot
def moveBaseSimple(msg):
    pass


# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    pass # Delete this 'pass' once implemented

# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('smchamberlain_lab3_node')


    # These are global variables. Write "global <variable_name>" in any other function
    #  to gain access to these global variables
    global calc_astar
    global pub
    global pose
    global odom_tf
    global odom_list
    global accum
    global Map
    
    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires

    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist) # Publisher for commanding robot motion

    gridCell_pub = rospy.Publisher('local_costmap/obstacles', GridCells) # Publisher for making grid cells

    point_pub = rospy.Publisher('/point', Point) 

    sub = rospy.Subscriber('/odom', Odometry, read_odometry, queue_size=1) # Callback function to read in robot Odometry messages

    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper',BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events

    map_sub = rospy.Subscriber('/map', OccupancyGrid, readMap, queue_size=1) #Callback function to handle mapping
 
    ipose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, readiPose, queue_size=1)#Callback Function to read initial robot position
   
    move_base_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, moveBaseSimple, queue_size=1)#Callback Function to move base?

    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    
    # Use this command to make the program wait for some seconds
    #rospy.sleep(rospy.Duration(2, 0))

    print "Starting Lab 3"
    print "%s" % Map.origin.position.x
    print "%s" % Map.origin.position.y
    # Make the robot do stuff...
    #rospy.spin()
    #make_obstacles()
    rospy.spin()
    print "Lab 3 complete!"
