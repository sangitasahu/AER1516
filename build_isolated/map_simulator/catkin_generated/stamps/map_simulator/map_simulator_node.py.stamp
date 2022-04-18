#!/usr/bin/env python

import rospy
from map_simulator.msg import Map3D
from geometry_msgs.msg import Vector3
import random

class mapSimulatorNode:
	def __init__(self):
		self.count = 0;
		self.NUM_OF_NODES = 5
		self.map = Map3D()
		self.vec3 = Vector3()
		self.freq = 10 #10 Hz
		self.mapPub = rospy.Publisher('internal_map', Map3D, queue_size=100)
		rospy.init_node('mapSimulatorNode', anonymous=True)

	def spin(self):
		self.rate = rospy.Rate(self.freq) 
		while not rospy.is_shutdown():
			self.map.nodeCoordinates.append(Vector3(random.uniform(-25,25),random.uniform(-15,15),random.uniform(-5,5)))			
			self.map.occupiedStatus.append((self.count % 3) == 1)
			self.map.nodeDistances.append(random.uniform(0,25))
			self.count += 1
			if self.count == self.NUM_OF_NODES:
				 rospy.loginfo(self.map)	
				 self.mapPub.publish(self.map)
				 self.count = 0
				 self.map.nodeCoordinates = []
				 self.map.occupiedStatus = []
				 self.map.nodeDistances = []
			self.rate.sleep()


if __name__ == '__main__' :
	try:
	 mapSimulatorNode = mapSimulatorNode()
	 mapSimulatorNode.spin()
	except rospy.ROSInterruptException:
	 pass
