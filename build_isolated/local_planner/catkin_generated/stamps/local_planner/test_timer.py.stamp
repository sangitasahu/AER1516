#!/usr/bin/env python

#
# AER 1516 Motion Planning Project
# Local Planner - Test Timer
#

# Import libraries
import rospy
import numpy as np

# Import message types
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Vector3, Quaternion, PoseStamped, Pose
from shape_msgs.msg import Plane
from std_msgs.msg import Float64, Header
from convex_decomposer.msg import CvxDecomp, Polyhedron

class local_planner_test_timer:
    def __init__(self):
        # Parameters
        self.loop_rate = 5 # Hz
        self.runnable_delay = 5 # s

        self.test_timer = rospy.Timer(rospy.Duration(1.0/self.loop_rate),self.runnable)

    def runnable(self,event):
        curr_time = rospy.get_rostime()
        # rospy.loginfo('Began runnable at {:.1f} s'.format(curr_time.to_sec()))
        print('Began runnable at {:.1f} s'.format(curr_time.to_sec()))
        rospy.sleep(self.runnable_delay)
        end_time = rospy.get_rostime()
        # rospy.loginfo('Ended runnable at {:.1f} s'.format(end_time.to_sec()))
        print('Ended runnable at {:.1f} s'.format(end_time.to_sec()))

if __name__ == '__main__' :
    try:
        # Initialize node
        rospy.init_node('local_planner_test_timer')
        local_planner_test_timer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass