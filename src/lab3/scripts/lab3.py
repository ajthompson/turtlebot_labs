#!/usr/bin/env python

import math
import rospy, tf
import roslib
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
import sys, select, termios, tty
from nav_msgs.msg import Odometry,OccupancyGrid

# Add additional imports for each of the message types used

   
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
def readiPose(msg)
    global initialPose
    
    initialpose = msg.PoseWithCovariance.pose
    
    print initialpose
pass

#Move Base Simple Callback Function to send endpoints to robot
def moveBaseSimple(msg)
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
    
    global pub
    global pose
    global odom_tf
    global odom_list
    global accum
    global Map
    
    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires

    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist) # Publisher for commanding robot motion

    sub = rospy.Subscriber('/odom', Odometry, read_odometry, queue_size=1) # Callback function to read in robot Odometry messages

    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper',BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events

    map_sub = rospy.Subscriber('/map', OccupancyGrid, readMap, queue_size=1) #Callback function to handle mapping
 
    ipose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, readiPose, queue_size=1)#Callback Function to read initial robot position
   
    move_base_sub = rospy.Subscriber(' /move_base_simple/goal', PoseStamped, moveBaseSimple, queue_size=1)#Callback Function to move base?

    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(2, 0))

    print "Starting Lab 3"
    
    # Make the robot do stuff...
    rospy.spin()
    print "Lab 3 complete!"
