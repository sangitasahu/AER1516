#!/usr/bin/env python
import rospy
from acl_msgs.msg import ViconState
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PointStamped
from acl_msgs.msg import FloatStamped
import numpy as np

IN_PROGRESS = 0
SUCCESS     = 1
FAIL        = 2

class collisionDetector:
	def __init__(self):
		self.init = False
		self.collisionDetected = False
		self.reachedGoal = False
		self.gotObstacles = False
		self.gotGoal = False
		self.poseSub = rospy.Subscriber('/LQ02s/vicon',ViconState, self.poseCB)
		self.obstSub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.obstCB)
		self.obstSub = rospy.Subscriber('/LQ02s/global_goal', PointStamped, self.goalCB)
		self.statusPub = rospy.Publisher("flight_status", FloatStamped, queue_size=1)
		self.obst = np.array([])
		self.init = True
		rospy.loginfo("Monitoring status...")

	def obstCB(self,data):
		if self.init:
			if not self.gotObstacles:
				for i in range(2,len(data.name)):
					if i==2:
						self.obst = np.array([data.pose[i].position.x,data.pose[i].position.y])
					else:
						self.obst = np.vstack((self.obst,np.array([data.pose[i].position.x,data.pose[i].position.y])))
				self.gotObstacles = True

	def poseCB(self,data):
		if self.init:
			if self.gotObstacles and not self.collisionDetected:
				self.pose = np.array([data.pose.position.x, data.pose.position.y])
				self.checkForCollision()

			if self.gotGoal:
				self.checkForGoal()

			self.publishStatus()

	def goalCB(self,data):
		if self.init:
			if not self.gotGoal:
				self.goal = np.array([data.point.x,data.point.y])
				self.gotGoal = True

	def publishStatus(self):
		if self.collisionDetected:
			self.status = FAIL
		elif self.reachedGoal:
			self.status = SUCCESS
		else:
			self.status = IN_PROGRESS

		flightStatus = FloatStamped()
		flightStatus.header.stamp = rospy.get_rostime()
		flightStatus.data = self.status

		self.statusPub.publish(flightStatus)

		if self.status != IN_PROGRESS:
			self.collisionDetected = False
			self.reachedGoal       = False
			self.gotObstacles      = False

	def checkForGoal(self):
		if np.linalg.norm(self.pose-self.goal) < 5:
			self.reachedGoal = True
			rospy.logwarn("Reached goal!")

	def checkForCollision(self):
		for i in range(0,len(self.obst)):
			if np.linalg.norm(self.pose-self.obst[i]) < 0.5:
				self.collisionDetected = True
				rospy.logfatal("Collision detected!")



if __name__ == '__main__':
	rospy.init_node("collisionCheck")
	c = collisionDetector()
	rospy.spin()