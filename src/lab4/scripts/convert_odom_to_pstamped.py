#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def readOdom(msg):
	convertedPose = PoseStamped()
	convertedPose.header = msg.header
	convertedPose.pose = msg.pose.pose
	pub.publish(convertedPose)
	print "Converted Pose at (%s, %s, %s)" % (convertedPose.pose.position.x, convertedPose.pose.position.y, convertedPose.pose.position.z)

if __name__ == '__main__':
	rospy.init_node('convert_init_pose')

	global pub

	pub = rospy.Publisher('/poseconv', PoseStamped)
	sub = rospy.Subscriber('/odom', Odometry, readOdom, queue_size=1)

	rospy.spin()