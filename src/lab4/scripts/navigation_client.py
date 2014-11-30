#!/usr/bin/env python
import math
import rospy, tf
import roslib
from lab4.srv import *
from lab4.msg import *
from lab3 import *
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped,Point
import sys, select, termios, tty
from nav_msgs.msg import Odometry,OccupancyGrid,GridCells,Path
import random


def run_navigation():
	global goal
	waitForGoal = 1

	while waitForGoal and not rospy.is_shutdown():
		try:
			goalPose = goal
			waitForGoal = 0
		except NameError:
			goalPose = None
			pass

	# Run for first goal
	compute_path()

	recalc = 0
	# loop to continue running
	while 1 and not rospy.is_shutdown():
		if goalPose != goal:
			recalc = 1
		if recalc:
			# run again
			compute_path()

# Add additional imports for each of the message types used
def compute_path():
	global path
	global pose
    
    initPose = copy.deepcopy(convertedPose)
	print "Entering loop to wait for positions"
	endWhileStart = 1
	endWhileGoal = 1
	# Make sure it doesn't start computation until both start and goal positions exist
	
	while (endWhileStart or endWhileGoal) and not rospy.is_shutdown():
		try:
			print pose
			startPose = initPose # should be current pose
			endWhileStart = 0
		except NameError:
			print "Start not found"
			startPose = None
			pass

		try:
			goalPose = goal
			endWhileGoal = 0
		except NameError:
			# print "Goal not found"
			# goalPose = None
			pass

	print "Found start and goal"
	print startPose.header.frame_id
	print startPose.pose.position.x
	print startPose.pose.position.y
	print goalPose.header.frame_id
	print goalPose.pose.position.x
	print goalPose.pose.position.y
	resp = calc_astar_client(startPose, goalPose)
	try:
		path = resp.path
	except:
		print "No path could be found"
	path_pub.publish(path)
	print "Finished calculation"

def calc_astar_client(start_pose, goal_pose):
	rospy.wait_for_service('calc_astar')
	try:
		calc_astar = rospy.ServiceProxy('calc_astar', Astar)
		resp1 = calc_astar(start_pose,goal_pose)
		return resp1
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
	except TypeError:
		print "Invalid start or goal position"
	except Empty:
		print "Unreachable goal position"
 
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
	print "Converting Pose"
	convertedPose = PoseStamped()
	convertedPose.header = msg.header
	convertedPose.pose = msg.pose.pose
	pose_stamped_pub.publish(convertedPose)

#Odometry Callback function.
def read_odometry(msg):
    global pose 
    global starter	
    global theta
    global cpose
    starter = msg
    pose = msg.pose.pose
    cpose = msg.pose.pose.orientation.z
    theta = math.asin(cpose)*2

def readConvPose(msg):
	global initialpose
	initialpose = msg

#Move Base Simple Callback Function to send endpoints to robot
def moveBaseSimple(msg):
    global goal
    print "Goal"
    goal = msg

def check_recalc(msg):
	recalc = msg

	if recalc:
		compute_path()
	else:
		# Do something when we finish
		pass

def pose_conv(msg)
    global convertedPose
    convertedPose= msg

# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    pass # Delete this 'pass' once implemented

# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('smchamberlain_lab4_node')


    # These are global variables. Write "global <variable_name>" in any other function
    #  to gain access to these global variables

    global calc_astar
    global pub
    global pose_stamped_pub
    global pose
    global odom_tf
    global odom_list
    global accum
    global Map
    global path

    # Subscribers
    sub = rospy.Subscriber('/odom', Odometry, read_odometry, queue_size=1) # Callback function to read in robot Odometry messages
    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper',BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    map_sub = rospy.Subscriber('/map', OccupancyGrid, readMap, queue_size=1) #Callback function to handle mapping
    ipose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, readiPose, queue_size=1)#Callback Function to read initial robot position
    converted_sub = rospy.Subscriber('/initialposeconv', PoseStamped, readConvPose, queue_size=1)
    goal_sub = rospy.Subscriber('/astar/goal', OccupancyGrid, moveBaseSimple, queue_size=1)
    recalc_sub = rospy.Subscriber('/recalc', Recalc, check_recalc, queue_size=1)

    # publishers
    pose_stamped_pub = rospy.Publisher('/initialposeconv', PoseStamped)
    inflated_ob_pub = rospy.Publisher('local_costmap/unknown_space',GridCells)
	path_pub = rospy.Publisher('/TrajectoryPlannerROS/global_plan',Path)
    ub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist) # Publisher for commanding robot motion
    gridCell_pub = rospy.Publisher('local_costmap/obstacles', GridCells) # Publisher for making grid cells
    point_pub = rospy.Publisher('/point', Point) 
    pose_conv_sub = rospy.Subscriber('/poseconv',PoseStamped, pose_conv)

    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    
    # Use this command to make the program wait for some seconds
    #rospy.sleep(rospy.Duration(2, 0))

    try:
    	test = Map
    except NameError:
    	Map = None

    while Map == None and not rospy.is_shutdown():
    	pass

    print "Starting Navigation Client"
    print "%s" % Map.origin.position.x
    print "%s" % Map.origin.position.y
    
    # Make the robot do stuff...
    #rospy.spin()
    #make_obstacles()
    compute_path()
    rospy.spin()

    print "Ending Navigation Client!"
