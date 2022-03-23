#!/usr/bin/env python

import rospy
from map_simulator.msg import Map3D

def readmap(map):
	rospy.loginfo("Map = ", map.nodeCoordinates , "Occ Sts =", map.occupiedStatus, "Dist Meas =", map.distances)


def globalPlanner():
	rospy.Subscriber("internal_map" , Map3D, readmap)
	rospy.spin()

if __name__ == '__main__':
	rospy.init_node('globalPlanner')
	globalPlanner()
