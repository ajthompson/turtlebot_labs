#!/usr/bin/env python

import rospy
from lab4.srv import *
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry,Path,OccupancyGrid
from kobuki_msgs.msg import BumperEvent
import tf
import numpy
import math 

#This function consumes linear and angular velocities
#and creates a Twist message.  This message is then published.
def publishTwist(lin_vel, ang_vel):
    global pub

    twist_msg = Twist();		#Create Twist Message

    twist_msg.linear.x = lin_vel	#Populate message with data
    twist_msg.angular.z = ang_vel

    #print "Publishing " + str(twist_msg)

    pub.publish(twist_msg)		#Send Message
 
#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    global odom_list
    global pose 	

    x0 = pose.pose.position.x	#Set origin
    y0 = pose.pose.position.y

    #Loop until the distance between the attached frame and the origin is equal to the
    #distance specifyed 
    done = False

    while (not done and not rospy.is_shutdown()):
        x1 = pose.pose.position.x
        y1 = pose.pose.position.y
        d = math.sqrt( (x1 - x0)**2 + (y1 - y0)**2 )	#Distance formula
        
        print "  " + str(distance) + " " + str(d)

        if (d >= distance):
            done = True
            publishTwist(0, 0)
        else:
            publishTwist(speed, 0)

        rospy.sleep(rospy.Duration(0, 500000))

    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    global odom_list
    global pose

    #This node was created using Coordinate system transforms and numpy arrays.
    #The goal is measured in the turtlebot's frame, transformed to the odom.frame 
    transformer = tf.TransformerROS()	
    rotation = numpy.array([[math.cos(angle), -math.sin(angle), 0],	#Create goal rotation
                            [math.sin(angle), math.cos(angle), 0],
                            [0,          0,          1]])

    #Get transforms for frames
    odom_list.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
    (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
    T_o_t = transformer.fromTranslationRotation(trans, rot)
    R_o_t = T_o_t[0:3,0:3]

    #Setup goal matrix
    goal_rot = numpy.dot(rotation, R_o_t)
    goal_o = numpy.array([[goal_rot[0,0], goal_rot[0,1], goal_rot[0,2], T_o_t[0,3]],
                    [goal_rot[1,0], goal_rot[1,1], goal_rot[1,2], T_o_t[1,3]],
                    [goal_rot[2,0], goal_rot[2,1], goal_rot[2,2], T_o_t[2,3]],
                    [0,             0,             0,             1]])

    #Continues creating and matching coordinate transforms.
    done = False
    while (not done and not rospy.is_shutdown()):
        (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        state = transformer.fromTranslationRotation(trans, rot)
        within_tolerance = abs((state - goal_o)) < .1
        if ( within_tolerance.all() ):
            spinWheels(0,0,0)
            done = True
        else:
            if (angle > 0):
                spinWheels(.1,-.1,.1)
            else:
                spinWheels(-.1,.1,.1)
	rospy.sleep(rospy.Duration(0, 500000))



#This function works the same as rotate how ever it does not publish linear velocities.
#Odometry Callback function.
def readOdom(msg):
    global pose
    global odom_tf

    pose = msg.pose
    geo_quat = pose.pose.orientation
  
    odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0),
        (pose.pose.orientation.x, pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w),
        rospy.Time.now(),
        "base_footprint",
        "odom")
    #odom_tf.sendTransform((pose.pose.position.x, pose,pose.position.y, 0), (pose.pose.orientation.x, pose.pose.orientation.y, \
#pose.pose.orientation.z, pose.pose.orientation.w), rospy.Time.now(), "base_footprint", "odom")

#The timer is used when it is saving data to csv files for analysis.
def timerCallback(event):
    global pose

    x = pose.pose.position.x
    y = pose.pose.position.y
    quaternion = (pose.pose.orientation.x,
                  pose.pose.orientation.y,
                  pose.pose.orientation.z,
                  pose.pose.orientation.w)

    euler = tf.transformations.euler_from_quaternion(quaternion)
    theta = euler[2]
def getObstacle(msg)

    global obstacle
    obstacle= msg



def adjustOrientation():
    global rotationMatrix
    global angle 
    angle = pose.pose.orientation.z
    rotationMatrix=np.matrix('math.cos(angle) -(math.sin(angle) 0; math.sin(angle) math.cos(angle) 0; 0 0 1')
    wayPointAngle = path_new.pose.orientation.z
    wayPointX = path_new.pose.position.x
    wayPointY = path_new.pose.position.y

    while (angle != wayPointAngle and rospy)and not rospy.is_shutdown():
        diff = angle - wayPointAngle
        rotate(diff)

        if diff < .0174532:
			
            break;

# Funtion to take in odometry and map and make robot move accordingly
def follow_path():
#take in path_new read in each index at a time 
# based on robot's current orientation and where grid cell is move robot
# if north northwest, so
    global x_pose
    global y_pose
    x_pose = pose.pose.position.x
    y_pose = pose.pose.position.y

	
    while (x_pose != wayPointX) and (y_pose != wayPointY) and not rospy.is_shutdown():
        adjustOrientation()
        dist=math.sqrt((x_pose - wayPointX)**2 + (y_pose - wayPointY)**2)
        driveStraight(.5,dist)
        dist=math.sqrt((x_pose - wayPointX)**2 + (y_pose - wayPointY)**2)
        recalc_msg = Recalc()
        newMsg = recalc_msg.recalculate 
        recalc_pub.publish(newMsg)
        print "Goal not achieved yet"
        
        if dist < .04:
            path_new.poses.pop
            recalc_pub.publish(newMsg)
            print "Subgoal achieved"
        break;
		            					
if __name__ == '__main__':
    try:
        print "Starting Lab 4"
        recalc_pub = Publisher.rospy('/recalc',Recalc) 
        path_sub = Subscriber.rospy('/path',Path,followPath)
        obstacle_sub = rospy.Subscriber('/expandedmap', OccupancyGrid, getObstacle)
        print "Lab4 Complete"

    except rospy.ROSInterruptException:
        pass
