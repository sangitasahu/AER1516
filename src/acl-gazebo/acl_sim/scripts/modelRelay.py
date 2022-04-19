#!/usr/bin/env python
# cartesian joystick waypoint control for quadrotor
import roslib
#roslib.load_manifest('quad_control')
import rospy
import copy
import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TwistStamped
from acl_msgs.msg import ViconState
from gazebo_msgs.msg import ModelState

CONTROL_DT = 0.01


class gazeboRelay():

    def __init__(self):
        self.gazebo_state = ModelState()
        self.vel = TwistStamped()
        self.pos = PoseStamped()
        self.pubState = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
        self.pubVel = rospy.Publisher('/clustering/true_vel',TwistStamped,queue_size=1)
        self.pubPos = rospy.Publisher('/clustering/true_pos',PoseStamped,queue_size=1)
        name = rospy.get_namespace()
        self.name = name[1:-1]

        rospy.Timer(rospy.Duration(CONTROL_DT),self.goalTimer)
        self.t0 = rospy.get_time()
        self.A = 2.0
        self.w = 1.0
        self.v = 4.0


    def goalTimer(self,e):
        t = rospy.get_time()-self.t0
        self.gazebo_state.model_name = self.name
        self.gazebo_state.pose.position.x = self.A*math.sin(self.w*t-4) + 4
        self.gazebo_state.pose.position.y = math.cos(self.w*t-1)
        self.gazebo_state.pose.position.z = self.A/2*math.sin(self.w*t-2) + 5
        self.gazebo_state.reference_frame = "world"   
        self.pubState.publish(self.gazebo_state)  

        self.pos.header.frame_id = "world"
        self.pos.header.stamp = rospy.get_rostime()
        self.pos.pose.position.x = self.A*math.sin(self.w*t-4) + 4
        self.pos.pose.position.y = math.cos(self.w*t-1)
        self.pos.pose.position.z = self.A/2*math.sin(self.w*t-2) + 5
        self.pubPos.publish(self.pos)     

        self.vel.header.frame_id = "world"
        self.vel.header.stamp = rospy.get_rostime()
        self.vel.twist.linear.x = self.A*self.w*math.cos(self.w*t-4)
        self.vel.twist.linear.y = -self.w*math.sin(self.w*t-1)
        self.vel.twist.linear.z = self.A/2*self.w*math.cos(self.w*t-2)
        self.pubVel.publish(self.vel)
        
        # t = rospy.get_time()-self.t0
        # if t > 2.5:
        #     t = 0
        #     self.t0 = rospy.get_time()

        # self.gazebo_state.model_name = self.name
        # self.gazebo_state.pose.position.x = 10 - self.v*t
        # self.gazebo_state.pose.position.y = -1 + 2*self.v/9*t
        # self.gazebo_state.pose.position.z = 4 + 2*self.v/9*t
        # self.gazebo_state.reference_frame = "world"   
        # self.pubState.publish(self.gazebo_state)       

        # self.vel.header.frame_id = "world"
        # self.vel.header.stamp = rospy.get_rostime()
        # self.vel.twist.linear.x = -self.v
        # self.vel.twist.linear.y = 2*self.v/9
        # self.vel.twist.linear.z = 2*self.v/9
        # self.pubVel.publish(self.vel)

def startNode():
    c = gazeboRelay()
    rospy.spin()

if __name__ == '__main__':

    ns = rospy.get_namespace()
    try:
        rospy.init_node('relay')
        if str(ns) == '/':
            rospy.logfatal("Need to specify namespace as vehicle name.")
            rospy.logfatal("This is tyipcally accomplished in a launch file.")
            rospy.logfatal("Command line: ROS_NAMESPACE=mQ01 $ rosrun quad_control joy.py")
        else:
            print "Starting joystick teleop node for: " + ns
            startNode()
    except rospy.ROSInterruptException:
        pass
