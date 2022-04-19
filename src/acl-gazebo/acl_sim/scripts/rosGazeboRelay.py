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
from acl_msgs.msg import ViconState
from gazebo_msgs.msg import ModelState


class QuadJoy:

    def __init__(self):
        self.gazebo_state = ModelState()
        self.pubState = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
        name = rospy.get_namespace()
        self.name = name[1:-1]


    def poseCB(self, data):
        self.gazebo_state.model_name = self.name
        self.gazebo_state.pose = data.pose
        self.gazebo_state.twist = data.twist
        self.gazebo_state.reference_frame = "world"   
        self.pubState.publish(self.gazebo_state)        

def startNode():
    c = QuadJoy()
    rospy.Subscriber("vicon", ViconState, c.poseCB)
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
