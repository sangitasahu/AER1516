#!/usr/bin/env python

import rospy
from map_simulator.msg import Map3D
import random

class mapSimulatorNode:
	def __init__(self):
		self.count = 0;
		self.NUM_OF_NODES = 49
		self.map = Map3D()
		self.map.nodeCoordinates = [0]*(3*self.NUM_OF_NODES)
		self.map.occupiedStatus = [False]*(self.NUM_OF_NODES)
		self.map.nodeDistances = [0]*(self.NUM_OF_NODES)
		self.freq = 10 #10 Hz
		self.mapPub = rospy.Publisher('internal_map', Map3D, queue_size=10)
		rospy.init_node('mapSimulatorNode', anonymous=True)

	def spin(self):
		self.rate = rospy.Rate(self.freq) 
		while not rospy.is_shutdown():
			self.map.nodeCoordinates[3*self.count + 0] = [random.randint(-25,25)]
			self.map.nodeCoordinates[3*self.count + 1] = [random.randint(-15,15)]
			self.map.nodeCoordinates[3*self.count + 2] = [random.randint(-5,5)]
			self.map.occupiedStatus[self.count] = ((self.count % 3) == 1)
			self.map.nodeDistances[self.count] = random.randint(0,25)
			self.count += 1
			rospy.loginfo(self.map)
			self.mapPub.publish(self.map)
			if self.count == self.NUM_OF_NODES:
				 self.count = 0
			self.rate.sleep()


if __name__ == '__main__' :
	try:
	 mapSimulatorNode = mapSimulatorNode()
	 mapSimulatorNode.spin()
	except rospy.ROSInterruptException:
	 pass
