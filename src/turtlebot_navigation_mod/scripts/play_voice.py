#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import Empty

def playVoice(msg):
	os.system("aplay '../sounds/heavy_specialcomplete01.wav'")


# This is the program's main function
if __name__ == '__main__':
	# Change this node name to include your username
	rospy.init_node('turtlebot_voice_node')
	rospy.spin()


	# Subscribers
	
	pose_sub = rospy.Subscriber('complete', Empty, playVoice, queue_size=1) 


