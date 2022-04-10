#!/usr/bin/env python

#
# AER 1516 Motion Planning Project
# Master Node
#

from __future__ import division, print_function, absolute_import

# Import libraries
import rospy
import numpy as np
import copy
from enum import IntEnum
from cmath import pi

# Import message types
from nav_msgs.msg import Path
from std_msgs.msg import UInt8, Header
from snapstack_msgs.msg import State, Goal
from geometry_msgs.msg import Point, PointStamped, Vector3, Quaternion, PoseStamped
from master_node.msg import MasterNodeState

class MasterNode(object):
    " Master node object for managing overall interactions, moding etc for motion planning project"
    def __init__(self):

        # Parameters
        # Flight Parameters
        self.start_delay = 3 # s
        self.start_x = 2
        self.start_y = 6
        self.start_z = 0
        self.start_yaw = 0
        self.flight_z = 1 # m. Fly at constant height off the ground for simplicity
        self.takeoff_speed = 1 # m/s

        self.frame_id = "world"
        
        # Modes
        # Goal Mode
        # 0 - Fixed goal location
        # 1 - Clicked point
        self.goal_mode = 0
        self.goal_fixed_x = 20
        self.goal_fixed_y = 20
        self.goal_fixed_z = self.flight_z
        self.goal_yaw = 0

        # Path mode
        # 0 - Local planner
        # 1 - Global planner passthrough (for debugging)
        # 2 - Hold location (for debugging)
        self.path_mode = 2
        self.global_plan_flight_speed = 3 # m/s

        # Rates
        self.goal_freq = 100 # Hz

        # Subscribers
        self.state_topic = '/SQ01s/state'
        self.state_sub = rospy.Subscriber(self.state_topic,State,callback=self.state_sub_callback)
        self.glob_plan_topic = 'global_plan'
        self.glob_plan_sub = rospy.Subscriber(self.glob_plan_topic,Path,callback=self.glob_plan_sub_callback)
        self.local_plan_goal_topic = 'local_plan_goal'
        self.local_plan_goal_sub = rospy.Subscriber(self.local_plan_goal_topic,Goal,callback=self.local_plan_sub_callback)
        self.clicked_point_topic = 'clicked_point'
        self.clicked_point_sub = rospy.Subscriber(self.clicked_point_topic,PointStamped,callback=self.clicked_point_sub_callback)

        # Publishers
        self.goal_topic = '/SQ01s/goal'
        self.goal_pub = rospy.Publisher(self.goal_topic,Goal,queue_size=10)
        self.goal_loc_topic = 'goal_loc'
        self.goal_loc_pub = rospy.Publisher(self.goal_loc_topic,PointStamped,queue_size=10)
        self.master_node_state = 'master_node_state'
        self.master_state_pub = rospy.Publisher(self.master_node_state,MasterNodeState,queue_size=10)

        # Timers
        self.update_goal_timer = rospy.Timer(rospy.Duration(1.0/self.goal_freq),self.update_goal_callback)

        # Storage
        # Inputs
        self.state = State()
        self.glob_plan = Path()
        self.clicked_point = PointStamped()
        self.local_plan_goal = Goal()

        # State variables
        self.node_state = NodeState.IDLE
        self.received_point = False
        self.received_local_plan = False
        self.received_global_plan = False
        self.start_timer = 0
        self.goal_filt_x = self.start_x
        self.goal_filt_y = self.start_y
        self.goal_filt_z = self.start_z
        self.goal_filt_yaw = self.start_yaw

        # Filter coefficients
        self.goal_filt_cutoff = 20 # Hz
        self.goal_filt_coef = (2*pi*self.goal_filt_cutoff/self.goal_freq)/(1+2*pi*self.goal_filt_cutoff/self.goal_freq)
        self.yaw_tol = 1E-4

    def state_sub_callback(self,msg):
        # TODO: May need to consider thread safety
        self.state = msg
    
    def glob_plan_sub_callback(self,msg):
        # TODO: May need to consider thread safety
        self.received_global_plan = True
        self.glob_plan = msg

    def clicked_point_sub_callback(self,msg):
        # TODO: May need to consider thread safety
        self.received_point = True
        self.clicked_point = msg

    def local_plan_sub_callback(self,msg):
        # TODO: May need to consider thread safety
        self.received_local_plan = True
        self.local_plan_goal = msg

    def update_goal_callback(self,event):
        # State machine based on node state
        if self.node_state == NodeState.IDLE:
            # Publish starting location until start time is met
            start_goal = Goal(header=Header(stamp=rospy.get_rostime(),frame_id = self.frame_id))
            start_goal.p = Vector3(x=self.start_x,y=self.start_y,z=self.start_z)
            self.goal_pub.publish(start_goal)

            self.start_timer += 1/(self.goal_freq)

            if self.start_timer>self.start_delay:
                # Transition to takeoff state
                self.node_state = NodeState.TAKEOFF

        elif self.node_state == NodeState.TAKEOFF:
            # Fake flying up to target z height
            # Incrementally update z position
            goal_z_temp = self.goal_filt_z+self.takeoff_speed/self.goal_freq
            if goal_z_temp > self.flight_z:
                # Reached target z height, transition to flight state
                self.goal_filt_z = self.flight_z
                if self.path_mode == 0:
                    self.node_state = NodeState.FLIGHT_LOCAL
                elif self.path_mode == 1:
                    self.node_state = NodeState.FLIGHT_GLOBAL
                else:
                    self.node_state = NodeState.FLIGHT_HOLD
            else:
                self.goal_filt_z = goal_z_temp
            
            # Publish updated goal location
            new_goal = Goal(header=Header(stamp=rospy.get_rostime(),frame_id = self.frame_id))
            new_goal.p = Vector3(x=self.start_x,y=self.start_y,z=self.goal_filt_z)
            new_goal.v = Vector3(z=self.takeoff_speed)
            self.goal_pub.publish(new_goal)

        elif self.node_state == NodeState.FLIGHT_LOCAL:
            # Pass through goal from local planner with low pass filter on positions/yaw to smooth any odd jumps
            # If unintialized, do not modify
            if self.received_local_plan:
                goal_new = copy.deepcopy(self.local_plan_goal)
                self.goal_filt_x = self.goal_filt_coef*goal_new.p.x + (1-self.goal_filt_coef)*self.goal_filt_x
                self.goal_filt_y = self.goal_filt_coef*goal_new.p.y + (1-self.goal_filt_coef)*self.goal_filt_y
                self.goal_filt_z = self.goal_filt_coef*goal_new.p.z + (1-self.goal_filt_coef)*self.goal_filt_z
                self.goal_filt_yaw = self.goal_filt_coef*goal_new.yaw + (1-self.goal_filt_coef)*self.goal_filt_yaw
                goal_new.p.x = self.goal_filt_x
                goal_new.p.y = self.goal_filt_y
                goal_new.p.z = self.goal_filt_z
                goal_new.yaw = self.goal_filt_yaw
                goal_new.header = Header(stamp=rospy.get_rostime(),frame_id = self.frame_id)

                # Publish new goal location
                self.goal_pub.publish(goal_new)

        elif self.node_state == NodeState.FLIGHT_GLOBAL:
            # Debugging mode to fly based on global planner only at a constant speed
            # Do nothing if haven't received global plan or list is zero length
            if self.received_global_plan and len(self.glob_plan.poses)>0:
                # Track along first leg of global plan
                c_mag = self.global_plan_flight_speed/self.goal_freq # Distance to travel per cycle
                curr_pos = np.array([self.state.pos.x,self.state.pos.y,self.state.pos.z])

                # Compute goal location
                if len(self.glob_plan.poses)==1:
                    # If list is only unit length, head towards start point
                    target_loc = np.array([self.glob_plan.poses[0].pose.position.x,
                                self.glob_plan.poses[0].pose.position.y,
                                self.glob_plan.poses[0].pose.position.z])
                    travel_dir = target_loc-curr_pos
                    if np.linalg.norm(travel_dir)<c_mag:
                        posn_reached = target_loc
                    else:
                        posn_reached = curr_pos+c_mag*travel_dir/np.linalg.norm(travel_dir)
                else:
                    # Project current position onto line
                    x_1 = np.array([self.glob_plan.poses[0].pose.position.x,
                                    self.glob_plan.poses[0].pose.position.y,
                                    self.glob_plan.poses[0].pose.position.z])
                    x_2 = np.array([self.glob_plan.poses[1].pose.position.x,
                                    self.glob_plan.poses[1].pose.position.y,
                                    self.glob_plan.poses[1].pose.position.z])
                    l = x_2-x_1
                    l_hat = l/max(1E-5,np.linalg.norm(l))
                    v = curr_pos-x_1
                    v_proj = v.dot(l_hat)*l_hat

                    # Try to track desired distance to a point along the first path segment
                    a = v_proj - v # Vector from current position to line
                    if np.linalg.norm(a) > c_mag:
                        # If we're too far away, drive laterally to the line. 
                        posn_reached = curr_pos+c_mag*a/np.linalg.norm(a)
                    else:
                        # Otherwise go as far as we can along it. Do not go past x_2
                        l_des = v.dot(l_hat) + np.sqrt(c_mag**2-a.dot(a))
                        l_clip = min(l_des,np.linalg.norm(l))
                        posn_reached = x_1 + l_clip*l_hat
                
                # Align yaw with heading direction, clamp when speed reaches zero
                head_dir = posn_reached-curr_pos
                if head_dir[0] < self.yaw_tol and head_dir[1] < self.yaw_tol:
                    yaw_targ = self.goal_filt_yaw
                else:
                    yaw_targ = np.arctan2(head_dir[1],head_dir[0])
                    
                # Apply first order lag filter to smooth transitions
                self.goal_filt_yaw = self.goal_filt_coef*yaw_targ + (1-self.goal_filt_coef)*self.goal_filt_yaw
                
                goal_new = Goal(header = Header(stamp=rospy.get_rostime(),frame_id = self.frame_id))
                goal_new.p.x = posn_reached[0]
                goal_new.p.y = posn_reached[1]
                goal_new.p.z = posn_reached[2]
                goal_new.yaw = self.goal_filt_yaw
                self.goal_pub.publish(goal_new)
        elif self.node_state == NodeState.FLIGHT_HOLD:
            # Hold position
            hold = 5
            
        # Update global goal location for global planner
        if self.goal_mode == 0:
            # Constant goal location
            goal_loc = PointStamped(header=Header(stamp=rospy.get_rostime(),frame_id = self.frame_id),
                                    point = Point(x=self.goal_fixed_x,y=self.goal_fixed_y,z=self.goal_fixed_z))
            self.goal_loc_pub.publish(goal_loc)
        else:
            # Clicked point control from RViz
            if self.received_point:
                # Update global goal with current clicked point value, held at target flight Z level
                global_goal = PointStamped(header=Header(stamp=rospy.get_rostime(),frame_id = self.frame_id),
                                    point = Point(x=self.clicked_point.point.x,y=self.clicked_point.point.y,z=self.clicked_point.point.z))
                global_goal.point.z = self.flight_z
                self.goal_loc_pub.publish(global_goal)
            else:
                # Have not received clicked point value, hold at start position
                global_goal = PointStamped(header=Header(stamp=rospy.get_rostime(),frame_id = self.frame_id),
                                    point = Point(x=self.start_x,y=self.start_y,z=self.flight_z))
                self.goal_loc_pub.publish(global_goal)
        
        # Publish master node state for reference by other components
        node_state = MasterNodeState(header=Header(stamp=rospy.get_rostime(),frame_id = self.frame_id),
                                     state = self.node_state)
        self.master_state_pub.publish(node_state)


class NodeState(IntEnum):
    IDLE = 0
    TAKEOFF = 1
    FLIGHT_LOCAL = 2
    FLIGHT_GLOBAL = 3
    FLIGHT_HOLD = 4
    UNKNOWN = 10

if __name__ == '__main__':
    try:
        # Initialize node
        rospy.init_node('master_node')
        MasterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
	    pass