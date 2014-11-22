import math
import rospy, tf
import roslib
from lab3.srv import *
from lab3 import astar_server
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped,Point
import sys, select, termios, tty
from nav_msgs.msg import Odometry,OccupancyGrid,GridCells,Path
import random

def expand_obstacles():
pass
# subscribe to map node
# expand obstacles through publishing inflated_obstacles
expanded = GridCells()
inflated_ob_pub.publish(expanded)



if __name__ == '__main__':

	inflated_ob_pub = rospy.Publisher('local_costmap/unknown_space',GridCells)

	gridCell_pub = rospy.Publisher('local_costmap/obstacles', GridCells) # Publisher for making grid cells

	map_sub = rospy.Subscriber('/map', OccupancyGrid, readMap, queue_size=1) #Callback function to handle mapping

	 ipose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, readiPose, queue_size=1)#Callback Function to read initial robot position

    pose_stamped_pub = rospy.Publisher('/initialposeconv', PoseStamped)

    converted_sub = rospy.Subscriber('/initialposeconv', PoseStamped, readConvPose, queue_size=1)

	expand_obstacles()



