#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

def readPose(msg):
	convertedPose = PoseStamped()
	convertedPose.header = msg.header
	convertedPose.pose = msg.pose.pose
	pub.publish(convertedPose)
	print "Converted Pose at (%s, %s, %s)" % (convertedPose.pose.position.x, convertedPose.pose.position.y, convertedPose.pose.position.z)

if __name__ == '__main__':
	rospy.init_node('convert_init_pose')

	global pub

	pub = rospy.Publisher('/initialposeconv', PoseStamped)
	sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, readPose, queue_size=1)

	rospy.spin()