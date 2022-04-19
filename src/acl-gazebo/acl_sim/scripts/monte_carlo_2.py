#!/usr/bin/env python
import rospy
from acl_msgs.msg import ViconState
from acl_msgs.msg import FloatStamped
from acl_msgs.msg import QuadFlightEvent
from geometry_msgs.msg import PointStamped
from gazebo_msgs.msg import ModelStates
import numpy as np
import os
import time

IN_PROGRESS = 0
SUCCESS     = 1
FAIL        = 2
TIME_OUT    = 3


class monteCarlo:
	def __init__(self):

		self.N = 48

		self.xmax = 50.0
		self.ymax = 25.0

		self.lam = 0.15

		self.t_max = 20

		self.memory = 0

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
		self.statusPub = rospy.Publisher("flight_status", FloatStamped, queue_size=1,latch=True)
		self.pubGoal = rospy.Publisher("/LQ02s/global_goal", PointStamped, queue_size=1)
		self.pubEvent = rospy.Publisher('/LQ02s/event',QuadFlightEvent,queue_size=1,latch=True)    

		rospy.Timer(rospy.Duration(0.1),self.goalTimer)

		self.openFile()
		self.start_process()
		self.spawn_field()
		self.start_up()


	def goalTimer(self,e):
		self.goal_pnt.header.stamp =rospy.get_rostime()
		self.goal_pnt.header.frame_id = "world"
		self.pubGoal.publish(self.goal_pnt)


	def poseCB(self,data):
		if self.init:
			self.pose = np.array([data.pose.position.x, data.pose.position.y])
			self.alt = data.pose.position.z

			if self.gotObstacles and not self.collisionDetected:
				self.checkForCollision()
				self.checkForGoal()

			self.publishStatus()


	def publishStatus(self):
		if self.flightevent.mode == self.flightevent.START:
			if (rospy.get_time() - self.t0) > self.t_max:
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
			self.t_goal = rospy.get_time() - self.t0
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
		if np.linalg.norm(self.pose-self.goal) < 2:
			self.reachedGoal = True
			rospy.logwarn("Goal reached!")

	def checkForCollision(self):
		for i in range(0,len(self.obst)):
			if np.linalg.norm(self.pose-self.obst[i]) < 0.45:
				self.collisionDetected = True
				rospy.logfatal("Collision detected!")


	def spawn_field(self):
		self.rho = np.random.poisson(self.lam*self.xmax*self.ymax)
		for i in range(0,self.rho):
			x = np.random.uniform(0,self.xmax)
			y = np.random.uniform(-self.ymax/2,self.ymax/2)

			if (i==self.numObstacles):
				self.obst = np.array([x,y])
			else:
				self.obst = np.vstack((self.obst,np.array([x,y])))

			os.system("rosrun gazebo_ros spawn_model -file `rospack find acl_sim`/models/cylinder/model.sdf -sdf -x " + str(x) + " -y " + str(y) + " -z 0 -model pole_"+str(i))
		
		os.system("clear")
		rospy.logwarn("Obstacles created")
		self.gotObstacles = True

	
	def delete_field(self):
		self.init = False
		self.collisionDetected = False
		self.reachedGoal = False
		self.gotObstacles = False
		self.gotGoal = False
		self.timeOut = False

		self.alt = 0

		# for i in range(0,self.rho):
		# 	os.system("rosservice call /gazebo/delete_model pole_"+str(i))
		# self.numObstacles += self.rho
		os.system("clear")
		rospy.logwarn("Obstacles deleted")


	def statusCB(self,data):
		# rospy.logfatal("msg: %0.2f",data.data)
		if data.data != IN_PROGRESS:
			self.count += 1
			if self.count < self.N+1:
				# kill controller and planner
				self.kill_process()
				# self.delete_field()
				self.start_process()
				if np.mod(self.count+1,2)==0:
					self.spawn_field()
				self.start_up()
			else:
				self.closeFile()
				self.kill_all_process()

	def kill_all(self):
		os.system("rosnode kill -a")

	def start_up(self):
		self.init = True
		rospy.logwarn("Starting simulation # %i of %i",self.count,self.N)
		rospy.sleep(2)
		self.flightevent.header.stamp = rospy.get_rostime()
		self.flightevent.mode = self.flightevent.TAKEOFF
		self.pubEvent.publish(self.flightevent)

		i = 0
		while (self.alt < self.goal_alt):
			# Do nothing
			i += 1

		self.flightevent.header.stamp = rospy.get_rostime()
		self.flightevent.mode = self.flightevent.START
		self.pubEvent.publish(self.flightevent)
		self.t0 = rospy.get_time()

	def kill_process(self):	
		
		self.init = False
		self.collisionDetected = False
		self.reachedGoal = False
		self.timeOut = False

		self.alt = 0

		os.system("rosnode kill /LQ02s/quad_sim /LQ02s/vicon_relay /LQ02s/relay /LQ02s/cntrl /LQ02s/tip_planner")
		if np.mod(self.count+1,2)==0:
			self.gotObstacles = False
			self.gotGoal = False
			os.system("killall -9 gzserver  & killall -9 gzclient")
			rospy.logfatal("All processes killed")
		else:
			rospy.logfatal("Restarting sim")


	def kill_all_process(self):
		self.kill_process()
		rospy.logfatal("Simulations complete")
		os.system("rosnode kill sim_master")

	def start_process(self):
		# rospy.sleep(1)
		if np.mod(self.count+1,2)==0:
			os.system("roslaunch acl_sim test.launch &")
		# rospy.sleep(2)
		os.system("ROS_NAMESPACE=LQ02s rosrun acl_demos rosGazeboRelay.py &")
		os.system("roslaunch quad_sim quad_sim.launch veh:=LQ num:=02 bag:=0 x:=-10 &")
		if np.mod(self.count,2)==0:
			self.memory = True
			os.system("roslaunch acl_planning tip.launch bag:=0 quad:=LQ02s memory:=true &")
		else:
			self.memory = False
			os.system("roslaunch acl_planning tip.launch bag:=0 quad:=LQ02s memory:=false &")


	def openFile(self):
		self.tfile_no = open("/home/brett/logs/test_log_" + time.strftime("%H-%M-%S_%d-%m-%Y") + "_mem_0_lam_"+str(self.lam)+".txt",'w')
		self.tfile_mem = open("/home/brett/logs/test_log_" + time.strftime("%H-%M-%S_%d-%m-%Y") + "_mem_1_lam_"+str(self.lam)+".txt",'w')

	def saveFile(self):
		if self.memory:
			if self.first_mem:
				self.tfile_mem.write("Iteration Result Obstacles Time \n")  
				self.first_mem = False
			self.tfile_mem.write("%i %i %i %0.4f  \n" % (self.count/2, self.status, self.rho, self.t_goal))
		else:
			if self.first_no:
				self.tfile_no.write("Iteration Result Obstacles Time \n")  
				self.first_no = False
			self.tfile_no.write("%i %i %i %0.4f  \n" % ((self.count+1)/2, self.status, self.rho, self.t_goal))

	def closeFile(self):
		self.tfile_mem.close()
		self.tfile_no.close()


if __name__ == '__main__':
	rospy.init_node('sim_master')
	c = monteCarlo()
	rospy.spin()