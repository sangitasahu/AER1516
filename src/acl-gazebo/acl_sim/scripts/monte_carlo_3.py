#!/usr/bin/env python
import rospy
from acl_msgs.msg import ViconState
from acl_msgs.msg import FloatStamped
from acl_msgs.msg import QuadFlightEvent
from geometry_msgs.msg import PointStamped
from gazebo_msgs.msg import ModelStates
import numpy as np
import scipy.spatial
import scipy as sp
import os
import time

IN_PROGRESS = 0
SUCCESS     = 1
FAIL        = 2
TIME_OUT    = 3


class monteCarlo:
	def __init__(self):

		self.N = 100

		self.xmax = 50.0
		self.ymax = 25.0

		self.lam = 0.2

		self.t_max = 30

		self.memory = 1

		self.count = 1

		self.init = False
		self.collisionDetected = False
		self.reachedGoal = False
		self.gotObstacles = False
		self.gotGoal = False
		self.timeOut = False
		self.first_mem = True
		self.first_no = True
		
		self.goal_pnt = PointStamped()
		self.flightevent = QuadFlightEvent()  
		self.flightevent.mode = self.flightevent.KILL

		self.goal_pnt.point.x = 60
		self.goal_pnt.point.y = 0
		self.goal_pnt.point.z = 1

		self.goal = np.array([self.goal_pnt.point.x, self.goal_pnt.point.y])

		self.alt = 0
		self.goal_alt = self.goal_pnt.point.z

		self.numObstacles = 0
		self.status_prev = 10    

		self.statusSub = rospy.Subscriber('flight_status', FloatStamped, self.statusCB)
		self.poseSub = rospy.Subscriber('/LQ02s/vicon',ViconState, self.poseCB)
		self.obstSub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.obstCB)
		self.statusPub = rospy.Publisher("flight_status", FloatStamped, queue_size=1,latch=True)
		self.pubGoal = rospy.Publisher("/LQ02s/global_goal", PointStamped, queue_size=1)
		self.pubEvent = rospy.Publisher('/LQ02s/event',QuadFlightEvent,queue_size=1,latch=True)    

		self.openFile()
		self.modXML()
		self.start_process()
		self.start_up()

	def obstCB(self,data):
		if self.init:
			if not self.gotObstacles:
				for i in range(2,len(data.name)):
					if i==2:
						self.obst = np.array([data.pose[i].position.x,data.pose[i].position.y])
						self.gotObstacles = True
					else:
						self.obst = np.vstack((self.obst,np.array([data.pose[i].position.x,data.pose[i].position.y])))
				if self.gotObstacles:			
					self.kdtree = sp.spatial.KDTree(self.obst,100)


	def poseCB(self,data):
		self.goal_pnt.header.stamp =rospy.get_rostime()
		self.goal_pnt.header.frame_id = "world"
		self.pubGoal.publish(self.goal_pnt)

		if self.init:
			self.pose = np.array([data.pose.position.x, data.pose.position.y])
			self.alt = data.pose.position.z

			if self.gotObstacles and not self.collisionDetected:
				self.checkForCollision()

			self.checkForGoal()

			self.publishStatus()


	def publishStatus(self):
		if self.flightevent.mode == self.flightevent.START:
			if (time.time() - self.t0) > self.t_max:
				self.timeOut = True
			else:
				self.timeOut = False

		if self.collisionDetected:
			self.status = FAIL
			self.t_goal = 1000
		elif self.timeOut:
			self.status = TIME_OUT
			self.t_goal = 1000
		elif self.reachedGoal:
			self.status = SUCCESS
			self.t_goal = time.time() - self.t0
		else:
			self.status = IN_PROGRESS

		flightStatus = FloatStamped()
		flightStatus.header.stamp = rospy.get_rostime()
		flightStatus.data = self.status

		if self.status_prev != self.status:
			if self.status != IN_PROGRESS:
				self.saveFile()

			self.statusPub.publish(flightStatus)
			self.status_prev = self.status


	def checkForGoal(self):
		if np.linalg.norm(self.pose-self.goal) < 2 or self.pose[0]>(self.goal[0]-5):
			self.reachedGoal = True
			rospy.logwarn("Goal reached!")

	def checkForCollision(self):
		d_closest, index_closest = self.kdtree.query(self.pose)
		if d_closest < 0.3:
			self.collisionDetected = True
			rospy.logfatal("Collision detected!")


	def statusCB(self,data):
		# rospy.logfatal("msg: %0.2f",data.data)
		if data.data != IN_PROGRESS:
			self.count += 1
			if self.count < self.N+1:
				# kill controller and planner
				self.kill_process()
				self.modXML()
				self.start_process()	
				self.start_up()
			else:
				self.closeFile()
				self.kill_all_process()


	def kill_all(self):
		os.system("rosnode kill -a")


	def start_up(self):
		self.init = True
		self.flightevent.header.stamp = rospy.get_rostime()
		self.flightevent.mode = self.flightevent.TAKEOFF
		self.pubEvent.publish(self.flightevent)

		i = 0
		while (self.alt < self.goal_alt):
			# Do nothing
			i += 1

		rospy.logwarn("Starting simulation # %i of %i",self.count,self.N)
		to = time.time()
		t = time.time()
		while (t-to<2):
			t = time.time()

		self.t0 = time.time()
		self.flightevent.header.stamp = rospy.get_rostime()
		self.flightevent.mode = self.flightevent.START
		self.pubEvent.publish(self.flightevent)


	def kill_process(self):			
		self.init = False
		self.status_prev = 10
		self.flightevent.mode = self.flightevent.KILL
		self.collisionDetected = False
		self.reachedGoal = False
		self.timeOut = False

		self.alt = 0

		os.system("rosnode kill /LQ02s/quad_sim /LQ02s/vicon_relay /LQ02s/relay /LQ02s/cntrl /LQ02s/tip_planner")
		# if np.mod(self.count+1,2)==0:
		if True:
			self.gotObstacles = False
			self.gotGoal = False
			os.system("killall -9 gzserver  & killall -9 gzclient")
			rospy.logfatal("All processes killed")
		else:
			rospy.logfatal("Restarting sim")

		to = time.time()
		t = time.time()
		while (t-to<2):
			t = time.time()


	def kill_all_process(self):
		self.kill_process()
		rospy.logfatal("Simulations complete")
		os.system("rosnode kill sim_master")


	def start_process(self):
		# if np.mod(self.count+1,2)==0:
		if True:
			os.system("roslaunch acl_sim test.launch &")
		os.system("ROS_NAMESPACE=LQ02s rosrun acl_sim rosGazeboRelay.py &")
		os.system("roslaunch quad_sim quad_sim.launch veh:=LQ num:=02 bag:=0 x:=-10 &")
		# if np.mod(self.count,2)==0:
		if True:
			self.memory = True
			os.system("roslaunch acl_planning tip.launch bag:=0 quad:=LQ02s memory:=true &")
		else:
			self.memory = False
			os.system("roslaunch acl_planning tip.launch bag:=0 quad:=LQ02s memory:=false &")


	def openFile(self):
		# self.tfile_no = open("/home/brett/logs/test_log_" + time.strftime("%H-%M-%S_%d-%m-%Y") + "_mem_0_lam_"+str(self.lam)+".txt",'w')
		self.tfile_mem = open("/home/brett/logs/test_log_" + time.strftime("%H-%M-%S_%d-%m-%Y") + "_mem_1_lam_"+str(self.lam)+".txt",'w')


	def saveFile(self):
		if self.memory:
			if self.first_mem:
				self.tfile_mem.write("Iteration Result Obstacles Time \n")  
				self.first_mem = False
			self.tfile_mem.write("%i %i %i %0.4f  \n" % (self.count, self.status, self.rho, self.t_goal))
		else:
			if self.first_no:
				self.tfile_no.write("Iteration Result Obstacles Time \n")  
				self.first_no = False
			self.tfile_no.write("%i %i %i %0.4f  \n" % ((self.count), self.status, self.rho, self.t_goal))


	def closeFile(self):
		# self.tfile_mem.close()
		self.tfile_no.close()


	def modXML(self):
		if True:
			self.rho = np.random.poisson(self.lam*self.xmax*self.ymax)		
			with open('../worlds/poisson_forest.world', 'r') as file:
			    # read a list of lines into data
			    data = file.readlines()

			data[94] = '<model_count>'+str(self.rho)+'</model_count> \n'

			with open('../worlds/poisson_forest.world', 'w') as file:
			    file.writelines( data )


if __name__ == '__main__':
	rospy.init_node('sim_master')
	c = monteCarlo()
	rospy.spin()