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
from std_msgs.msg import UInt8, Header, Float64
from snapstack_msgs.msg import State, Goal
from geometry_msgs.msg import Point, PointStamped, Vector3, Quaternion, Pose, PoseStamped
from tf.transformations import euler_from_quaternion
from master_node.msg import MasterNodeState

class MasterNode(object):
    " Master node object for managing overall interactions, moding etc for motion planning project"
    def __init__(self):

        # Parameters
        # Flight Parameters
        self.start_delay = rospy.get_param("~master/start_delay") # s
        self.start_x = rospy.get_param("~master/start_x")
        self.start_y = rospy.get_param("~master/start_y")
        self.start_z = rospy.get_param("~master/start_z")
        self.start_yaw = rospy.get_param("~master/start_yaw")
        self.flight_z = rospy.get_param("~master/flight_z") # m. Fly at constant height off the ground for simplicity
        self.takeoff_speed = 3 # m/s

        self.frame_id = "world"
        
        # Modes
        # Goal Mode
        # 0 - Fixed goal location
        # 1 - Nav goal
        self.goal_mode = rospy.get_param("~master/goal_mode")
        self.goal_fixed_x = rospy.get_param("~master/goal_x")
        self.goal_fixed_y = rospy.get_param("~master/goal_y")
        self.goal_fixed_z = rospy.get_param("~master/goal_z")
        self.goal_yaw = 0

        # Path mode
        # 0 - Local planner
        # 1 - Global planner passthrough (for debugging)
        # 2 - Hold location (for debugging)
        self.path_mode = rospy.get_param("~master/path_mode")
        self.global_plan_flight_speed = rospy.get_param("~master/global_plan_v") # m/s

        # Clicked point unstuck mode
        self.enable_click_unstuck = 0

        # Rates
        self.goal_freq = 100 # Hz

        use_JPS3D = rospy.get_param("~setup/use_jps3d")

        # Subscribers
        self.state_topic = '/SQ01s/state'
        self.state_sub = rospy.Subscriber(self.state_topic,State,callback=self.state_sub_callback)
        if use_JPS3D:
            self.glob_plan_topic = '/SQ01s/faster/global_plan'
        else:
            self.glob_plan_topic = 'global_plan'
        self.glob_plan_sub = rospy.Subscriber(self.glob_plan_topic,Path,callback=self.glob_plan_sub_callback)
        self.local_plan_goal_topic = '/local_planner/local_plan_goal'
        self.local_plan_goal_sub = rospy.Subscriber(self.local_plan_goal_topic,Goal,callback=self.local_plan_sub_callback)

        self.clicked_point_topic = '/clicked_point'
        self.clicked_point_sub = rospy.Subscriber(self.clicked_point_topic,PointStamped,callback=self.clicked_point_sub_callback)
        self.nav_goal_topic = '/move_base_simple/goal'
        self.nav_goal_sub = rospy.Subscriber(self.nav_goal_topic,PoseStamped,callback=self.nav_goal_sub_callback)

        # Publishers
        self.goal_topic = '/SQ01s/goal'
        self.goal_pub = rospy.Publisher(self.goal_topic,Goal,queue_size=10)
        self.goal_loc_topic = 'goal_loc'
        self.goal_loc_pub = rospy.Publisher(self.goal_loc_topic,PoseStamped,queue_size=10)
        self.master_node_state = 'master_node_state'
        self.master_state_pub = rospy.Publisher(self.master_node_state,MasterNodeState,queue_size=10)
        self.path_length_topic = '/data/path_length'
        self.path_length_pub = rospy.Publisher(self.path_length_topic,Float64,queue_size=10)
        self.applied_vel_topic = '/data/applied_vel'
        self.applied_vel_pub = rospy.Publisher(self.applied_vel_topic,Vector3,queue_size=10)

        # Timers
        self.update_goal_timer = rospy.Timer(rospy.Duration(1.0/self.goal_freq),self.update_goal_callback)

        # Storage
        # Inputs
        self.state = State()
        self.glob_plan = Path()
        self.clicked_point = PointStamped()
        self.nav_goal = PoseStamped()
        self.local_plan_goal = Goal()

        # State variables
        self.node_state = NodeState.IDLE
        self.received_point = False
        self.received_nav_goal = False
        self.received_first_nav_goal = False
        self.received_local_plan = False
        self.received_global_plan = False
        self.start_timer = 0
        self.goal_filt_x = self.start_x
        self.goal_filt_y = self.start_y
        self.goal_filt_z = self.start_z
        self.goal_filt_yaw = self.start_yaw

        self.clicked_point_last_x = 0
        self.clicked_point_last_y = 0
        self.clicked_point_last_yaw = 0

        # Filter coefficients
        self.goal_filt_cutoff = 20 # Hz
        self.goal_filt_coef = (2*pi*self.goal_filt_cutoff/self.goal_freq)/(1+2*pi*self.goal_filt_cutoff/self.goal_freq)
        self.goal_filt_cutoff_global = 2 # Hz
        self.goal_filt_coef_global = (2*pi*self.goal_filt_cutoff_global/self.goal_freq)/(1+2*pi*self.goal_filt_cutoff_global/self.goal_freq)
        self.yaw_tol = 1E-12

        # Data Logging
        self.path_length = 0
        self.applied_v_x = 0
        self.applied_v_y = 0
        self.applied_v_z = 0

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

    def nav_goal_sub_callback(self,msg):
        # TODO: May need to consider thread safety
        if not self.received_first_nav_goal:
            self.received_first_nav_goal = True
        self.received_nav_goal = True
        self.nav_goal = msg

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
            start_goal.yaw = self.start_yaw
            self.goal_pub.publish(start_goal)

            self.start_timer += 1/(self.goal_freq)

            if self.start_timer>self.start_delay:
                # Transition to takeoff state
                self.node_state = NodeState.TAKEOFF
                rospy.loginfo("Transitioning to takeoff state")

        elif self.node_state == NodeState.TAKEOFF:
            # Fake flying up to target z height
            # Incrementally update z position
            goal_z_temp = self.goal_filt_z+self.takeoff_speed/self.goal_freq
            if goal_z_temp > self.flight_z:
                # Reached target z height, transition to flight state
                self.goal_filt_z = self.flight_z
                if self.path_mode == 0:
                    self.node_state = NodeState.FLIGHT_LOCAL
                    rospy.loginfo("Transitioning to local planner flight state")
                elif self.path_mode == 1:
                    self.node_state = NodeState.FLIGHT_GLOBAL
                    rospy.loginfo("Transitioning to global planner flight state")
                else:
                    self.node_state = NodeState.FLIGHT_HOLD
                    rospy.loginfo("Transitioning to hold state")
            else:
                self.goal_filt_z = goal_z_temp
            
            # Publish updated goal location
            new_goal = Goal(header=Header(stamp=rospy.get_rostime(),frame_id = self.frame_id))
            new_goal.p = Vector3(x=self.start_x,y=self.start_y,z=self.goal_filt_z)
            new_goal.yaw = self.start_yaw
            if self.node_state == NodeState.TAKEOFF:
                new_goal.v = Vector3(z=self.takeoff_speed)
            self.goal_pub.publish(new_goal)

        elif self.node_state == NodeState.FLIGHT_LOCAL:
            # Pass through goal from local planner with low pass filter on positions/yaw to smooth any odd jumps
            # If unintialized, do not modify
            if self.received_local_plan:
                goal_new = copy.deepcopy(self.local_plan_goal)
                # print('I read local goal: [{:.2f},{:.2f},{:.2f}]'.format(goal_new.p.x,goal_new.p.y,goal_new.p.z))
                delta_x = self.goal_filt_coef*(goal_new.p.x-self.goal_filt_x)
                delta_y = self.goal_filt_coef*(goal_new.p.y-self.goal_filt_y)
                delta_z = self.goal_filt_coef*(goal_new.p.z-self.goal_filt_z)
                # self.goal_filt_x = self.goal_filt_coef*goal_new.p.x + (1-self.goal_filt_coef)*self.goal_filt_x
                # self.goal_filt_y = self.goal_filt_coef*goal_new.p.y + (1-self.goal_filt_coef)*self.goal_filt_y
                # self.goal_filt_z = self.goal_filt_coef*goal_new.p.z + (1-self.goal_filt_coef)*self.goal_filt_z
                self.goal_filt_x = self.goal_filt_x + delta_x
                self.goal_filt_y = self.goal_filt_y + delta_y
                self.goal_filt_z = self.goal_filt_z + delta_z
                self.goal_filt_yaw = self.goal_filt_coef*goal_new.yaw + (1-self.goal_filt_coef)*self.goal_filt_yaw
                goal_new.p.x = self.goal_filt_x
                goal_new.p.y = self.goal_filt_y
                goal_new.p.z = self.goal_filt_z
                goal_new.yaw = self.goal_filt_yaw
                goal_new.header = Header(stamp=rospy.get_rostime(),frame_id = self.frame_id)

                # Publish new goal location
                self.goal_pub.publish(goal_new)

                # Data logging
                self.applied_v_x = delta_x*self.goal_freq
                self.applied_v_y = delta_y*self.goal_freq
                self.applied_v_z = delta_z*self.goal_freq
                self.path_length = self.path_length + np.sqrt(delta_x**2+delta_y**2+delta_z**2)

                self.path_length_pub.publish(Float64(data=self.path_length))
                self.applied_vel_pub.publish(Vector3(x = self.applied_v_x,y = self.applied_v_y,z = self.applied_v_z))

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
                    # Find line that we're closest to
                    a_best = 1E6
                    ind_best = 0
                    for i in range(len(self.glob_plan.poses)-1):
                        # Project current position onto line
                        x_1 = np.array([self.glob_plan.poses[i].pose.position.x,
                                        self.glob_plan.poses[i].pose.position.y,
                                        self.glob_plan.poses[i].pose.position.z])
                        x_2 = np.array([self.glob_plan.poses[i+1].pose.position.x,
                                        self.glob_plan.poses[i+1].pose.position.y,
                                        self.glob_plan.poses[i+1].pose.position.z])
                        l = x_2-x_1
                        l_hat = l/max(1E-5,np.linalg.norm(l))
                        v = curr_pos-x_1
                        v_proj = v.dot(l_hat)*l_hat
                        a_mag = np.linalg.norm(v_proj - v) # Vector from current position to line

                        if a_mag<a_best or (a_mag<1E-10):
                            # Grab highest index that we're closest to
                            a_best = a_mag
                            ind_best = i

                    # Track along line we're closest to
                    # Project current position onto line
                    x_1 = np.array([self.glob_plan.poses[ind_best].pose.position.x,
                                        self.glob_plan.poses[ind_best].pose.position.y,
                                        self.glob_plan.poses[ind_best].pose.position.z])
                    x_2 = np.array([self.glob_plan.poses[ind_best+1].pose.position.x,
                                    self.glob_plan.poses[ind_best+1].pose.position.y,
                                    self.glob_plan.poses[ind_best+1].pose.position.z])
                    l = x_2-x_1
                    l_hat = l/max(1E-5,np.linalg.norm(l))
                    v = curr_pos-x_1
                    v_proj = v.dot(l_hat)*l_hat
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
                if abs(head_dir[0]) < self.yaw_tol and abs(head_dir[1]) < self.yaw_tol:
                    yaw_targ = self.goal_filt_yaw
                else:
                    yaw_targ = np.arctan2(head_dir[1],head_dir[0])
                    
                # Apply first order lag filter to smooth transitions
                self.goal_filt_yaw = self.goal_filt_coef_global*yaw_targ + (1-self.goal_filt_coef_global)*self.goal_filt_yaw
                
                goal_new = Goal(header = Header(stamp=rospy.get_rostime(),frame_id = self.frame_id))
                goal_new.p.x = posn_reached[0]
                goal_new.p.y = posn_reached[1]
                goal_new.p.z = posn_reached[2]
                goal_new.yaw = self.goal_filt_yaw
                self.goal_pub.publish(goal_new)
        elif self.node_state == NodeState.FLIGHT_HOLD:
            # Hold position at nav goal
            if self.received_nav_goal:
                goal_new = Goal(header = Header(stamp=rospy.get_rostime(),frame_id = self.frame_id))
                goal_new.p.x = self.nav_goal.pose.position.x
                goal_new.p.y = self.nav_goal.pose.position.y
                goal_new.p.z = self.flight_z
                quat_clicked = [self.nav_goal.pose.orientation.x,self.nav_goal.pose.orientation.y,
                            self.nav_goal.pose.orientation.z,self.nav_goal.pose.orientation.w]
                euler_clicked = euler_from_quaternion(quat_clicked,'rzyx')
                goal_new.yaw = euler_clicked[0]
                self.goal_pub.publish(goal_new)
                self.received_nav_goal = False

        # Update global goal location for global planner
        if (self.master_node_state == NodeState.IDLE or self.master_node_state == NodeState.TAKEOFF):
            # Publish a bogus starting goal location to make FASTER node happy
            global_goal = PoseStamped(header=Header(stamp=rospy.get_rostime(),frame_id = self.frame_id))
            global_goal.pose = Pose(position = Point(x=self.start_x,y=self.start_y,z=self.flight_z),
                            orientation = Quaternion(x = 0, y = 0, z = 0, w = 1))
            self.goal_loc_pub.publish(global_goal)
        else:
            if self.goal_mode == 0:
                # Constant goal location
                # goal_loc = PointStamped(header=Header(stamp=rospy.get_rostime(),frame_id = self.frame_id),
                #                         point = Point(x=self.goal_fixed_x,y=self.goal_fixed_y,z=self.goal_fixed_z))
                goal_loc = PoseStamped(header=Header(stamp=rospy.get_rostime(),frame_id = self.frame_id))
                goal_loc.pose = Pose(position = Point(x=self.goal_fixed_x,y=self.goal_fixed_y,z=self.goal_fixed_z),
                                    orientation = Quaternion(x = 0, y = 0, z = 0, w = 1))
                self.goal_loc_pub.publish(goal_loc)
            else:
                # Clicked point control from RViz
                if self.received_first_nav_goal:
                    if self.received_nav_goal:
                        # Update global goal with current clicked point value, held at target flight Z level
                        # global_goal = PointStamped(header=Header(stamp=rospy.get_rostime(),frame_id = self.frame_id),
                        #                     point = Point(x=self.nav_goal.pose.position.x,
                        #                     y=self.nav_goal.pose.position.y,z=self.nav_goal.pose.position.z))
                        global_goal = self.nav_goal
                        global_goal.header = Header(stamp=rospy.get_rostime(),frame_id = self.frame_id)
                        global_goal.pose.position.z = self.flight_z
                        self.goal_loc_pub.publish(global_goal)
                        self.received_nav_goal = False
                else:
                    # Have not received clicked point value, hold at start position
                    # global_goal = PointStamped(header=Header(stamp=rospy.get_rostime(),frame_id = self.frame_id),
                    #                     point = Point(x=self.start_x,y=self.start_y,z=self.flight_z))
                    global_goal = PoseStamped(header=Header(stamp=rospy.get_rostime(),frame_id = self.frame_id))
                    global_goal.pose = Pose(position = Point(x=self.start_x,y=self.start_y,z=self.flight_z),
                                    orientation = Quaternion(x = 0, y = 0, z = 0, w = 1))
                    self.goal_loc_pub.publish(global_goal)
        
        if self.enable_click_unstuck:
            if self.received_point:
                # Override commanded position with clicked point in case get stuck
                print("Unstucking")
                goal_new = Goal()
                goal_new.p.x = self.clicked_point.point.x
                goal_new.p.y = self.clicked_point.point.y
                goal_new.p.z = self.flight_z

                dx = self.clicked_point.point.x-self.state.pos.x
                dy = self.clicked_point.point.y-self.state.pos.x

                if abs(dx)<self.yaw_tol and abs(dy)<self.yaw_tol:
                    quat = [self.state.quat.x,self.state.quat.y,
                            self.state.quat.z,self.state.quat.w]
                    euler = euler_from_quaternion(quat,'rzyx')
                    goal_new.yaw = euler[0]
                else:
                    goal_new.yaw = np.arctan2(dy,dx)
                
                self.goal_pub.publish(goal_new)

                self.received_point = False

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