#!/usr/bin/env python

import rospy
from map_simulator.msg import Map3D
from geometry_msgs.msg import Vector3
from snapstack_msgs.msg import Goal
from snapstack_msgs.msg import State
from trajectory_msgs.msg import JointTrajectoryPoint


class globalPlanner:
	def __init__(self):
		self.num_of_cells = 0 
		self.cur_goal = Goal()
		self.cur_state = State()
		self.map = Map3D()
		self.global_traj = JointTrajectoryPoint()
		self.global_traj_pub = rospy.Publisher('global_trajectory', JointTrajectoryPoint, queue_size=10)
	
	def spin(self):
	   	rospy.Subscriber("/SQ01s/goal" , Goal, self.read_goal)
		rospy.Subscriber("/SQ01s/state" , State, self.read_state)
		rospy.Subscriber("internal_map" , Map3D, self.read_map)
		self.global_traj = self.generate_global_traj()
		self.global_traj_pub.publish(self.global_traj)
		rospy.spin()
	
	def read_map(self,msg):
		self.map = msg	
		rospy.loginfo(map.nodeCoordinates)
		rospy.loginfo(map.occupiedStatus)
		rospy.loginfo(map.nodeDistances)
		rospy.loginfo(self.cur_goal.p)
		rospy.loginfo(self.cur_state.pos)		

	def read_goal(self,msg):
		self.cur_goal = msg;

	def read_state(self,msg):
		self.cur_state = msg;
	
	def generate_global_traj(self):		
		self.global_traj.positions = [self.map.nodeCoordinates[i] for i in range(len(self.map.nodeCoordinates))]
		self.global_traj.velocities = [0 for i in range(len(self.map.nodeCoordinates))]
		self.global_traj.accelerations = [0 for i in range(len(self.map.nodeCoordinates))] 
		self.global_traj.effort = [0 for i in range(len(self.map.nodeCoordinates))] 
		self.global_traj.time_from_start = 0

if __name__ == '__main__':
	rospy.init_node('globalPlanner')
	globalPlanner_o = globalPlanner()
	globalPlanner_o.spin()
