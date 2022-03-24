#!/usr/bin/env python

#
# AER 1516 Motion Planning Project
# Local Planner
#

from __future__ import division, print_function, absolute_import
from cmath import pi

# Import libraries
import rospy
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis, quaternion_multiply

# Import message types
from nav_msgs.msg import Path
from snapstack_msgs.msg import State, Goal
from geometry_msgs.msg import Point, Vector3, Quaternion, PoseStamped
from shape_msgs.msg import Plane
from std_msgs.msg import Float64, Header
from local_planner.msg import CvxDecomp, Polyhedron

class LocalPlanner(object):
    "Local Planner Class"
    def __init__(self):
        
        # Storage
        # Inputs
        self.state = State()
        self.glob_plan = Path()
        self.cvx_decomp = CvxDecomp()
        self.global_goal = Point()

        # Outputs
        self.goal = Goal()
        self.goal.p.x=0.0
        self.goal.p.y=0.0
        self.goal.p.z=0.0
        self.goal.v.x=0.0
        self.goal.v.y=0.0
        self.goal.v.z=0.0
        self.goal.a.x=0.0
        self.goal.a.y=0.0
        self.goal.a.z=0.0

        self.local_plan = Path()

        # State Variables
        self.state_pos = Vector3()
        self.state_vel = Vector3()
        self.state_quat = Quaternion()
        self.glob_plan_poses = [PoseStamped]

        # Start time
        self.t_start = rospy.get_rostime()

    def replan(self):
        potato = 5

    def update_goal(self):
        potato = 5

    def replan_debug(self):
        # Print interfaces to demonstrate they're coming in properly
        rospy.loginfo("Global Plan Position 1: %s",self.glob_plan.poses[0].pose.position)
        rospy.loginfo("Global Plan Position 2: %s",self.glob_plan.poses[1].pose.position)
        
        plane_1 = self.cvx_decomp.polyhedra[0].planes[0]
        plane_2 = self.cvx_decomp.polyhedra[0].planes[1]

        rospy.loginfo("Plane 1: %s",plane_1)
        rospy.loginfo("Plane 2: %s",plane_2)
        
        # a := coef[0]
        # b := coef[1]
        # c := coef[2]
        # d := coef[3]

        # float64[4] coef

    def update_goal_debug(self):
        # Fudge the goal location
        curr_time = rospy.get_rostime()
        dt = curr_time-self.t_start
        sine_freq = 0.5 # Hz
        sine_amp = 2 # m?
        self.goal.p.x = 0
        self.goal.p.y = 0
        self.goal.p.z = sine_amp*np.sin(2*pi*sine_freq*dt.to_sec())
        self.goal.a.y = 10*np.sin(2*2*pi*sine_freq*dt.to_sec())
        #rospy.loginfo("Applied z position %.3f m",self.goal.p.z)
        self.goal.header.frame_id = "map"
        self.goal.header.stamp = rospy.get_rostime()

        # Draw a sinusoidal path. S for snake
        new_path = Path()
        pose_list = []
        for i in range(6):
            made_up_pose = PoseStamped()
            made_up_pose.header.stamp = curr_time
            made_up_pose.header.frame_id = "map"
            made_up_pose.pose.position.x = 0.4*i
            made_up_pose.pose.position.y = ((0.4*i)**2)*np.sin(2*pi*sine_freq*dt.to_sec())
            made_up_pose.pose.position.z = 0
            pose_list.append(made_up_pose)
        new_path.poses = pose_list
        new_path.header.stamp = curr_time
        new_path.header.frame_id = "map"
        rospy.loginfo("Path has y position %.3f m",new_path.poses[5].pose.position.y)
        self.local_plan = new_path

if __name__ == '__main__':
    pass