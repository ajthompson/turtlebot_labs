#!/usr/bin/env python
import math
import rospy
import copy
from lab3.srv import *
from final.srv import *
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from move_base_msgs import MoveBaseActionFeedback, MoveBaseActionGoal
from actionlib_msgs import GoalStatusArray, GoalStatus
#spin
#frontiers
#astar
#piblish message

# bumper spin and recalulate

#Service Proxy?

def run_navigation(bumper):
	global finished
	global goal_counter
	global centroids

	goal_counter += 1

	# spin if not the bumper
	if not bumper:
		rotate(2 * math.pi)

	# calculate the frontiers
	centroids = calc_frontier_client().centroids

	if len(centroids) > 0:
		# get the first centroid
		first = centroids.pop(0)
		# create a goal
		goal = MoveBaseActionGoal()
		goal.header = first.header
		goal.goal_id.id = "Goal%s" % goal_counter
		goal.goal.target_pose = first
		goal_pub.publish(goal)
	else:	# there are no reachable centroids
		finished = 1

# Add additional imports for each of the message types used
def compute_path():
	global path
	global path_pub
	global pose
	global newPose
	initPose = copy.deepcopy(newPose)
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
		path_pub.publish(path)
	except:
		print "No path could be found"
	
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

def calc_frontier_client():
	rospy.wait_for_service('calc_frontier')
	try:
		calc_astar = rospy.ServiceProxy('calc_frontier', Frontier)
		resp1 = calc_frontier()
		return resp1
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
	except TypeError:
		print "Invalid start or goal position"

def rotate(angle):
    print "Begin to rotate"
    global odom_list
    global pose
    

    global accum
    rate = rospy.Rate(100)
 
    twist=Twist()
    #startpose=pose
    accum= 0
    speed = .5
    last = theta_from_quat(pose)
    if angle < 0:
        angle = angle * -1
        speed = speed * -1
    # while math.fabs(pose.orientation.z - startpose.orientation.z) < angle and not rospy.is_shutdown():  
    while accum < angle and not rospy.is_shutdown():
        accum=accum + math.fabs(theta_from_quat(pose) - last)
       
        #print accum
        last = theta_from_quat(pose)
        if (abs(accum - angle) < 0.5):
            twist.angular.z = speed / 2
        else:
            twist.angular.z = speed
        pub.publish(twist)
        rate.sleep()
    twist.angular.z = 0
    pub.publish(twist)

def pose_conv(msg):
	global newPose
	newPose = msg

def feedback(msg):
	global turtle_feedback
	turtle_feedback= msg

def status(msg):
	global turtle_status
	turtle_status = msg

def result(msg):
	global centroids
	turtle_result = msg
	result_status = turtle_result.status

	# determine what to do dependant on the status given in the message
	# A lot of these are the same, but they may change if the nav stack is found to not
	# be good enough. Only check if goal id matches counter
	if result_status.goal_id.id == goal_counter:
		if result_status.status == GoalStatus.PENDING:		# The goal has not yet been processed by server
			print "Goal %s pending" % goal_counter
		elif result_status.status == GoalStatus.ACTIVE:		# Goal is currently being executed
			print "Goal %s active"
		elif result_status.status == GoalStatus.PREEMPTED:	# The goal was canceled during execution
			print "Goal %s preempted" % goal_counter
			run_navigation(False)
		elif result_status.status == GoalStatus.SUCCEEDED:	# The goal was reached
			print "Goal %s reached" % goal_counter
			run_navigation(False)
		elif result_status.status == GoalStatus.ABORTED:	# The goal was aborted (Stuck/fail to reach)
			print "Goal %s aborted" % goal_counter
			run_navigation(False)
		elif result_status.status == GoalStatus.REJECTED:	# The goal was determined unreachable by the nav stack
			print "Goal %s rejected" % goal_counter
			run_navigation(False) 
		elif result_status.status == GoalStatus.PREEMPTING:	# The goal received a cancel request during execution
			print "Goal %s preempting" % goal_counter
		elif result_status.status == GoalStatus.RECALLING:	# Request to cancel received, not confirmed as dead
			print "Goal %s recalling" % goal_counter
		elif result_status.status == GoalStatus.RECALLED:	# Goal successfully canceled before execution
			print "Goal %s recalled" % goal_counter
		elif result_status.status == GoalStatus.LOST:		# Goal lost, shouldn't occur ever
			print "Goal %s lost" % goal_counter
			print "\tSomeone ignored the documentation..."

#Bumper Event Callback function
def readBumper(msg):
    # run_navigation(True)

# This is the program's main function
if __name__ == '__main__':
	# Change this node name to include your username
	rospy.init_node('turtlebot_client_node')

	global finished
	global path_pub
	global twist_pub
	global goal_counter

	# publishers
	# path_pub = rospy.Publisher('/TrajectoryPlannerROS/global_plan',Path)
	goal_pub = rospy.Publisher('/move_base/goal')
	twist_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist) # Publisher for commanding robot motion

	# Subscribers
	feedback_sub = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback,feedback, queue_size=1)
	status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, status, queue_size=1)
	result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, result, queue_size=1)
	bumper_sub = rospy.Subscriber('/mobile_base/events/bumper',BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
	pose_conv_sub = rospy.Subscriber('/poseconv',PoseStamped, pose_conv, queue_size=1)

	print "Turtlebot SLAM Begin"
	finished = 0
	goal_counter = -1
	run_navigation(False)
	
	# wait for the program to finish
	while not finished and not rospy.is_shutdown():
		rospy.sleep(rospy.Duration(1))

	print "Turtlebot SLAM is complete"
	print "EXITING"