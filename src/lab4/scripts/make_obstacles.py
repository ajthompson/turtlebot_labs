import math
import rospy, tf
import roslib
from lab3.srv import *
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped,Point
import sys, select, termios, tty
from nav_msgs.msg import Odometry,OccupancyGrid,GridCells,Path
import random

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
    
