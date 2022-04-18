#!/usr/bin/env python2

#
# AER 1516 Motion Planning Project
# Local Planner Node
#

from __future__ import division, print_function, absolute_import

# Import libraries
import rospy
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis, quaternion_multiply

from local_planner_class import LocalPlanner

# Import message types
from nav_msgs.msg import Path
from snapstack_msgs.msg import State, Goal
from geometry_msgs.msg import Point, Vector3, Quaternion, PoseStamped, PointStamped
from std_msgs.msg import Float64
from convex_decomposer.msg import CvxDecomp, Polyhedron
from master_node.msg import MasterNodeState

class LocalPlannerNode(object):
    " Local planner node object for optimization based local trajectory planning"
    def __init__(self):

        # Rates
        self.replan_freq = 10
        self.goal_freq = 100
        self.fake_dynamics_freq = 100

        # Objects
        self.local_planner = LocalPlanner(self.replan_freq,self.goal_freq,self.fake_dynamics_freq)

        # Subscribers
        self.state_topic = '/SQ01s/state'
        self.state_sub = rospy.Subscriber(self.state_topic,State,callback=self.state_sub_callback)
        self.glob_plan_topic = 'global_plan'
        self.glob_plan_sub = rospy.Subscriber(self.glob_plan_topic,Path,callback=self.glob_plan_sub_callback)
        self.cvx_decomp_topic = 'CvxDecomp'
        self.cvx_decomp_sub = rospy.Subscriber(self.cvx_decomp_topic,CvxDecomp,callback=self.cvx_decomp_sub_callback)
        self.global_goal_topic = 'goal_loc'
        self.global_goal_sub = rospy.Subscriber(self.global_goal_topic,PointStamped,callback=self.global_goal_sub_callback)
        self.master_node_state_topic = 'master_node_state'
        self.master_node_state_sub = rospy.Subscriber(self.master_node_state_topic,MasterNodeState,callback=self.master_node_state_sub_callback)

        # Publishers
        self.local_goal_topic = 'local_plan_goal'
        self.local_goal_pub = rospy.Publisher(self.local_goal_topic,Goal,queue_size=10)
        self.path_topic = 'local_plan'
        self.path_pub = rospy.Publisher(self.path_topic,Path,queue_size=10)

        # Timers
        self.replan_timer = rospy.Timer(rospy.Duration(1.0/self.replan_freq),self.replan_callback)
        self.update_goal_timer = rospy.Timer(rospy.Duration(1.0/self.goal_freq),self.update_goal_callback)
        self.fake_dynamics_timer = rospy.Timer(rospy.Duration(1.0/self.fake_dynamics_freq),self.fake_dynamics_callback)

    def state_sub_callback(self,msg):
        # TODO: May need to consider thread safety
        self.local_planner.state = msg
        if not self.local_planner.received_state:
            self.local_planner.received_state = True
    
    def glob_plan_sub_callback(self,msg):
        # TODO: May need to consider thread safety
        self.local_planner.glob_plan = msg
        if not self.local_planner.received_glob_plan:
            self.local_planner.received_glob_plan = True

    def cvx_decomp_sub_callback(self,msg):
        # TODO: May need to consider thread safety
        self.local_planner.cvx_decomp = msg
        if not self.local_planner.received_cvx_decomp:
            self.local_planner.received_cvx_decomp = True

    def global_goal_sub_callback(self,msg):
        # TODO: May need to consider thread safety
        self.local_planner.global_goal = msg
        if not self.local_planner.received_global_goal:
            self.local_planner.received_global_goal = True

    def master_node_state_sub_callback(self,msg):
        # TODO: May need to consider thread safety
        self.local_planner.master_node_state = msg

    def replan_callback(self,event):
        # Execute replanning step and publish path for visualization
        self.local_planner.replan()
        if self.local_planner.opt_run:
            self.path_pub.publish(self.local_planner.local_plan)

    def update_goal_callback(self,event):
        # Interpolate goal at current point in time and publish
        self.local_planner.update_goal()
        if self.local_planner.opt_run:
            self.local_goal_pub.publish(self.local_planner.goal)

    def fake_dynamics_callback(self,event):
        # Interpolate goal at current point in time and publish
        self.local_planner.fake_dynamics()

if __name__ == '__main__':
    try:
        # Initialize node
        rospy.init_node('local_planner')
        LocalPlannerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
	    pass