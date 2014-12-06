#!/usr/bin/env python
import rospy, math, copy
import numpy as np
import numpy.ma as ma
from final.srv import *
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, GridCells

# Each cells contains the x and y index in the map
class Cell:
	def __init__(self, x, y):
		self.x = x
		self.y = y

	# overload equality method
	def __eq__ (self, other):
		return self.x == other.x and self.y == other.y

	# Create string method to print the cell
	def __str__ (self):
		return "(%s, %s)" % (self.x, self.y)

# contains the list of frontier ids and the list of cells contained on this frontier
class Frontier:

	def __init__(self, f_id, first_cell_x, first_cell_y):
		self.size = 0
		self.f_ids = list()
		self.f_ids.append(f_id)
		self.cells = list()
		self.add_cell(first_cell_x, first_cell_y)

	# overload equality method
	def __eq__ (self, other):
		# select shorter list
		ids_to_check = self.f_ids if len(self.f_ids) < len(other.f_ids) else other.f_ids
		list_to_check = other.f_ids if len(self.f_ids) < len(other.f_ids) else self.f_ids
		# iterate checking if list_to_check contains any ids in ids_to_check
		for i in ids_to_check:
			if i in list_to_check:
				return True
		return False

	def __cmp__ (self, other):
		return self.size - other.size

	def __str__ (self):
		title = "Frontier of size %s:\n" % self.size

		ids = "\tIDs: "
		for i in self.f_ids:
			ids += "%s " % i
		ids += "\n"

		cells = "\tCells: "
		for j in self.cells:
			cells += "%s " % j
		cells += "\n"
		return title + ids + cells

	def add_cell(self, x, y):
		self.cells.append(Cell(x, y))
		self.size += 1

	def merge(self, other):
		# merge the frontier ids
		# don't need to check for overlap, as merge is only used on bordering frontiers
		for i in other.f_ids:
			self.f_ids.append(i)

		# merge the cells
		for j in other.cells:
			self.cells.append(j)

		# add the sizes
		self.size += other.size

if __name__ == "__main__":
	global costmap
	global frontiers

	frontier_server()
