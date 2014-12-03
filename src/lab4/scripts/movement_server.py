#!/usr/bin/env python

import rospy
from lab4.srv import *
from lab4.msg import *
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry,Path,OccupancyGrid
from kobuki_msgs.msg import BumperEvent
import tf
import numpy
import math 
import copy

#This function consumes linear and angular velocities
#and creates a Twist message.  This message is then published.
def publishTwist(lin_vel, ang_vel):
    global pub

    twist_msg = Twist();        #Create Twist Message

    twist_msg.linear.x = lin_vel    #Populate message with data
    twist_msg.angular.z = ang_vel

    #print "Publishing " + str(twist_msg)

    pub.publish(twist_msg)        #Send Message
 
#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    global pose
    prev_pos = copy.deepcopy(pose)
    twist = Twist()
    twist.linear.y = 0
    twist.linear.z = 0
    accum = 0
    while accum < distance and not rospy.is_shutdown():
        current_pose = copy.deepcopy(pose)
        accum += math.sqrt((current_pose.pose.position.x - prev_pos.pose.position.x)**2 + (current_pose.pose.position.y - prev_pos.pose.position.y)**2)
        prev_pos = current_pose
        pub.publish(twist)
        if (abs(accum - distance) < 0.25):
            twist.linear.x = speed / 2
        else:
            twist.linear.x = speed

    twist.linear.x = 0
    pub.publish(twist)
    
#Accepts an angle and makes the robot rotate around it.
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
  #  while math.fabs(pose.orientation.z - startpose.orientation.z) < angle and not rospy.is_shutdown():  
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
   


#This function works the same as rotate how ever it does not publish linear velocities.
#Odometry Callback function.
def spinWheels(u1, u2, time):
    global pub

    lin_vel = 0.5 * (u1 + u2)            #Determines the linear velocity of base based on the wheels
    ang_vel = (1 / .352) * (u1 - u2)        #Determines the angular velocity of base bread on the wheels.

    twist_msg = Twist();            #Creates two messages: a name-maker and a program killer
    stop_msg = Twist();

    twist_msg.linear.x = lin_vel        #Populate messages with data.
    twist_msg.angular.z = ang_vel
    stop_msg.linear.x = 0
    stop_msg.angular.z = 0
    
    #While the specified amount of time has not elapsed, send Twist messages.
    now = rospy.Time.now().secs
    while (rospy.Time.now().secs - now <= time and not rospy.is_shutdown()):
        pub.publish(twist_msg)
    pub.publish(stop_msg)
    

#This function accepts a speed and a distance for the robot to move in a straight line
def readOdom(msg):
    global pose
    global odom_tf
    global theta

    try:
        pose = msg.pose
        geo_quat = pose.pose.orientation

        quaternion = (pose.pose.orientation.x,
                      pose.pose.orientation.y,
                      pose.pose.orientation.z,
                      pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        theta = euler[2]
      
        odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0),
            (pose.pose.orientation.x, pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w),
            rospy.Time.now(),
            "base_footprint",
            "odom")
    except NameError:
        pass

    # convert to euler_coord
    #odom_tf.sendTransform((pose.pose.position.x, pose,pose.position.y, 0), (pose.pose.orientation.x, pose.pose.orientation.y, \
#pose.pose.orientation.z, pose.pose.orientation.w), rospy.Time.now(), "base_footprint", "odom")

#The timer is used when it is saving data to csv files for analysis.
def theta_from_quat(in_val):
        quaternion = (in_val.pose.orientation.x,
                      in_val.pose.orientation.y,
                      in_val.pose.orientation.z,
                      in_val.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler[2]

def timerCallback(event):
    pass

def getObstacle(msg):
    global obstacle
    obstacle = msg


def adjustOrientation():
    global rotationMatrix
    #global angle 
    angle = theta_from_quat(pose)
    # print "Euler Angle: %s" % angle
    #rotationMatrix=numpy.matrix('math.cos(angle) -(math.sin(angle) 0; math.sin(angle) math.cos(angle) 0; 0 0 1')
    wayPointAngle = math.atan2(new_pose.pose.position.y - pose.pose.position.y,new_pose.pose.position.x - pose.pose.position.x)

    while wayPointAngle < -math.pi:
        wayPointAngle += 2 * math.pi
    while wayPointAngle >= math.pi:
        wayPointAngle -= 2 * math.pi

    # print "WayPoint Angle: %s" %wayPointAngle
    #wayPointX = new_pose.pose.position.x
    #wayPointY = new_pose.pose.position.y
    diff = 0.2
    while (angle != wayPointAngle and rospy)and not rospy.is_shutdown():
        if abs(diff) < 0.1 or abs(diff - 2*math.pi) < 0.1 or abs(diff + 2*math.pi) < 0.1:        #original value: .1    
            break;
        angle = theta_from_quat(pose)
        diff = wayPointAngle - angle
        # print "angle %s\nwayPoint %s\ndiff %s" % (angle, wayPointAngle, diff)
        # print "Angle to rotate: %s" % diff
        rotate(diff)

# Funtion to take in odometry and map and make robot move accordingly
def follow_path(msg):
    print "BEGIN TO FOLLOW PATH"
#take in path_new read in each index at a time 
# based on robot's current orientation and where grid cell is move robot
# if north northwest, so
    global path_new
    global new_pose
    global x_pose
    global y_pose
    global recalc_flag
    global finished
    recalc_msg = Recalc()
    recalc_msg.recalculate = 0    

    path_new = copy.deepcopy(msg)

    recalc_flag = 0
    finished = 0
    index = 1

    while index < len(path_new.poses):
        x_pose = pose.pose.position.x
        y_pose = pose.pose.position.y

        new_pose = path_new.poses[index]
        
        goalPoseX = path_new.poses[-1].pose.position.x
        goalPoseY = path_new.poses[-1].pose.position.y
    
        while (x_pose != goalPoseX) and (y_pose != goalPoseY) and not rospy.is_shutdown():
            wayPointX = new_pose.pose.position.x
            wayPointY = new_pose.pose.position.y
            print "\tDestination (%s, %s)" % (wayPointX, wayPointY)
            dist = math.sqrt((x_pose - wayPointX)**2 + (y_pose - wayPointY)**2)
            print "\tAdjusting orientation"
            # turn to face the waypoint
            adjustOrientation()

            rospy.sleep(rospy.Duration(30))
            # calculate distance to the waypoint
            dist = math.sqrt((x_pose - wayPointX)**2 + (y_pose - wayPointY)**2)
            print "\tCalculating distance as %s" % dist
            if check_path(new_pose):
                print "\t\tPath blocked, recalculate"
                recalc_flag = 1
                break
                # path is blocked, recalculate
            # drive the distance
            driveStraight(.1,dist)
            x_pose = pose.pose.position.x
            y_pose = pose.pose.position.y
            dist = math.sqrt((x_pose - wayPointX)**2 + (y_pose - wayPointY)**2)
            if dist < 0.10:
                if wayPointX == goalPoseX and wayPointY == goalPoseY:
                    finished = 1
                    break
                else:
                    index += 1
                    new_pose = path_new.poses[index]
                    print "Waypoint achieved"
            else:
                print "\t\tError too large, recalculate"
                recalc_flag = 1
                break



        if recalc_flag or finished:
            break

    if recalc_flag == 1:
        print "\tSending recalc message"
        recalc_msg.recalculate = 1
    elif finished == 1:
        print "\tSending complete message"
        recalc_msg.recalculate = 0    
    recalc_pub.publish(recalc_msg)

def check_path(new_pose):
    print "Checking path"
    # extract info from the expanded map
    ex_map = copy.deepcopy(obstacle)
    res = ex_map.info.resolution
    width = ex_map.info.width
    height = ex_map.info.height
    offset = ex_map.info.origin

    print "Offset = (%s, %s)" % (offset.position.x, offset.position.y)

    startPose = copy.deepcopy(pose)

    # get initial and goal indices
    init_x = int(math.floor((startPose.pose.position.x - offset.position.x + res / 2) / res))
    init_y = int(math.floor((startPose.pose.position.y - offset.position.y + res / 2) / res))

    goal_x = int(math.floor((new_pose.pose.position.x - offset.position.x + res / 2) / res))
    goal_y = int(math.floor((new_pose.pose.position.y - offset.position.y + res / 2) / res))

    # get angle
    angle = math.atan2(goal_y - init_y, goal_x - init_x)
    print "Path angle %s" % angle

    # calculate the increments
    x_inc = 0.1 / res * math.cos(angle)
    y_inc = 0.1 / res * math.sin(angle)
    print "Increment X by %s" % x_inc
    print "Increment Y by %s" % y_inc

    i = 0
    # set to impossible values
    next_i_x = math.ceil(width * 100 * res)  # i is in units of gridcells with 1 being
    next_i_y = math.ceil(height * 100 * res) # equal to 1 gridcell/(100*resolution)

    # we know these aren't occupied, because we're there
    current_x = init_x
    current_y = init_y

    # set x and y +1 sign
    if angle >= -math.pi and angle < -math.pi / 2:    # 3rd quadrant
        x_p = -1
        y_p = -1
    elif angle >= -math.pi / 2 and angle < 0:        # 4th quadrant
        x_p = 1
        y_p = -1
    elif angle >= 0 and angle < math.pi / 2:        # 1st quadrant
        x_p = 1
        y_p = 1
    elif angle >= math.pi / 2 and angle <= math.pi:    # 2nd quadrant
        x_p = -1
        y_p = 1

    print "Start:\n\tx: %s\n\ty: %s" % (current_x, current_y)
    print "Goal:\n\tx: %s\n\ty: %s" % (goal_x, goal_y)

    stuck = 0

    while 1 and not rospy.is_shutdown() and current_x >= 0 and current_x < width and current_y >= 0 and current_y < height:
        new_x = int(math.floor(i * x_inc + init_x))
        new_y = int(math.floor(i * y_inc + init_y))
        print "\tChecking (%s, %s)" % (new_x, new_y)

        if new_x == goal_x and new_y == goal_y:
            print "\tFinished"
            current_x = new_x
            current_y = new_y
            if current_x < width and current_x >= 0 and current_y < height and current_y >= 0:
                if ex_map.data[current_y*width + current_x] >= 100:
                    return True
            return False

        elif new_x == current_x and new_y == current_y:
            print "\tSame cell"
            i += 1
        else:
            print "\tDifferent cell"
            current_x = new_x
            current_y = new_y
            if current_x < width and current_x >= 0 and current_y < height and current_y >= 0:
                if ex_map.data[current_y*width + current_x] >= 50:
                    # cell is occupied, recalculate
                    return True
            i += 1

    # while 1 and not rospy.is_shutdown():
    #     new_x = int(math.floor(i * x_inc + init_x))
    #     new_y = int(math.floor(i * y_inc + init_y))
    #     print "\tChecking (%s, %s)" % (new_x, new_y)

        # # check against previous
        # if new_x == goal_x and new_y == goal_y:
        #     # check the goal position
        #     if ex_map.data[new_y*width + new_x] >= 50:
        #         # cell is occupied, recalculate
        #         return True
        #     return False
        # elif new_x == current_x and new_y == current_y:
        #     print "\t\tSame cell"
        #     # same grid, calculate new i
        #     i = i + 1
        #     # if stuck:
        #     #     print "\t\t\tStuck, incrementing i"
        #     #     i = i + 0.1
        #     #     #print "i: %s" % i
        #     # else:
        #     #     print "\t\t\tRecalculating i"
        #     #     if x_inc > 0.01:
        #     #         next_i_x = (new_x + x_p - init_x) / x_inc
        #     #     if y_inc > 0.01:
        #     #         next_i_y = (new_y + y_p - init_y) / y_inc

        #     #     i = min(next_i_x, next_i_y)
        #     #     print "\t\t\t\tNext i = %s" % i
        #     # stuck = 1
        # # elif new_x != current_x and new_y == current_y:
        # #     print "X changed"
        # #     current_x = new_x
        # #     # change in x axis, check new cell
        # #     if current_x < width and current_y < width * height:
        # #         if ex_map.data[current_y*width + current_x] >= 50:
        # #             # cell is occupied, recalculate
        # #             return True
        # #     stuck = 0
        # #     # calculate the new counter value - we already know the next y position
        # #     if x_inc > 0.01:
        # #         next_i_x = (new_x + x_p - init_x) / x_inc
        # #     i = min(next_i_x, next_i_y)
        # # elif new_x == current_x and new_y != current_y:
        # #     print "Y changed"
        # #     current_y = new_y
        # #     # change in y axis, check new cell
        # #     if current_x < width and current_y < width * height:
        # #         if ex_map.data[current_y*width + current_x] >= 50:
        # #             # cell is occupied, recalculate
        # #             return True

        # #     # calculate the new counter value - we already know the next x position
        # #     if y_inc > 0.01:
        # #         next_i_y = (new_y + y_p - init_y) / y_inc
        # #     i = min(next_i_x, next_i_y)
        # #     stuck = 0
        # else:
        #     print "\t\tBoth changed"
        #     # both changed
        #     current_x = new_x
        #     current_y = new_y

        #     if current_x < width and current_y < width * height:
        #         if ex_map.data[current_y*width + current_x] >= 50:
        #             # cell is occupied, recalculate
        #             return True

        #     # if x_inc > 0.01:
        #     #     next_i_x = (new_x + x_p - init_x) / x_inc
        #     # if y_inc > 0.01:
        #     #     next_i_y = (new_y + y_p - init_y) / y_inc
        #     # i = min(next_i_x, next_i_y)
        #     i = i + 1
        #     stuck = 0
                                
if __name__ == '__main__':
    global odom_tf
    global odom_list

    rospy.init_node('movement_node')

    try:
        print "Starting Lab 4"
        pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)
        recalc_pub = rospy.Publisher('/recalc',Recalc) 
    
        path_sub = rospy.Subscriber('/TrajectoryPlannerROS/global_plan', Path, follow_path)
        obstacle_sub = rospy.Subscriber('/expandedmap', OccupancyGrid, getObstacle)

        pose_sub = rospy.Subscriber('/odom', Odometry, readOdom, queue_size=1) 

        odom_list = tf.TransformListener()
        odom_tf = tf.TransformBroadcaster()
        
        rospy.spin()
        print "Lab4 Complete"

    except rospy.ROSInterruptException:
        pass
