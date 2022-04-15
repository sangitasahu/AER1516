#!/usr/bin/env python

#
# AER 1516 Motion Planning Project
# Fake Global Planner for testing Convex Decomposition + Local Planner
#

# Import libraries
import rospy
import numpy as np

# Import message types
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Vector3, Quaternion, PoseStamped, Pose
from snapstack_msgs.msg import State, Goal
from std_msgs.msg import Float64, Header
from master_node.msg import MasterNodeState

class fake_global_planner:
    def __init__(self):
        # Subscribers
        self.state_topic = '/SQ01s/state'
        self.state_sub = rospy.Subscriber(self.state_topic,State,callback=self.state_sub_callback)
        self.master_node_state_topic = 'master_node_state'
        self.master_node_state_sub = rospy.Subscriber(self.master_node_state_topic,MasterNodeState,callback=self.master_node_state_sub_callback)

        self.state = State()
        self.master_node_state = MasterNodeState()

        self.received_state = False

        # Publishers
        self.glob_plan_topic = 'global_plan'
        self.glob_plan_pub = rospy.Publisher(self.glob_plan_topic,Path,queue_size=10)

        # Timers
        self.global_plan_freq = 20 # Hz
        self.global_plan_timer = rospy.Timer(rospy.Duration(1.0/self.global_plan_freq),self.replan)
        
        # Fixed paths available
        plan_1 = [[0, 0, 2],
                #[6.951447010040283, 1.953622817993164, 2],
                [6.951447010040283, 2.75, 2],
                [7.402541160583496, 5.609556198120117, 2],
                [10.79484748840332, 13.773212432861328, 2],
                # [16.73023223876953, 15.89638900756836, 2],
                [17.503000259399414, 17.28727912902832, 2],
                [25.346839904785156, 21.625633239746094, 2]]

        plan_2 = [[0, 0, 2],
                [12.999226570129395, 2.9544198513031006, 2],
                # [20.09488868713379, 5.923097133636475, 2],
                [20.44375991821289, 5.5183634757995605, 2],
                # [28.212825775146484, 7.473572254180908, 2],
                [26.698017120361328, 6.988714694976807, 2],
                [27.69146156311035, 12.604181289672852, 2],
                [23.996479034423828, 16.840625762939453, 2],
                [25.48357582092285, 21.48044776916504, 2]]

        plan_3 = [[0, 0, 2],
                [1.0713133811950684, 5.005500793457031, 2],
                [6.565757751464844, 10.081217765808105, 2],
                [2.0662026405334473, 13.586685180664062, 2],
                [1.94272780418396, 18.80951499938965, 2],
                [7.015960693359375, 21.648677825927734, 2],
                [10.520651817321777, 20.46749496459961, 2],
                [12.725071907043457, 24.69696807861328, 2],
                [18.308774948120117, 28.512062072753906, 2]]

        # Pick your poison
        self.global_plan_len = 1
        self.global_plan_base = plan_3

        # Indicators if we've visited a point on the plan
        self.visited = [False]*len(self.global_plan_base)
        self.visited[0] = True
        self.plan_ind_curr = 0
        self.dist_next_wp_prev = 10000
        self.dist_next_wp_prev_tol = 1E-4
        self.visit_tol = 2 # m

        # 0 - Reach mode. Use for plan length = 1
        # 1 - Drive by mode. Use for plan length >1
        self.next_wp_mode = 0

        # Debug
        self.verbose = True
        self.report_counter = 0
        self.report_interval = 50

    def replan(self,event):
        
        # Need inputs initialized
        if not (self.received_state and self.master_node_state.state == self.master_node_state.FLIGHT_LOCAL):
            return

        curr_pos = [self.state.pos.x,self.state.pos.y,self.state.pos.z]
        next_wp = self.global_plan_base[self.plan_ind_curr+1]
        dist_next_wp = np.sqrt((curr_pos[0]-next_wp[0])**2+
                                (curr_pos[1]-next_wp[1])**2+
                                (curr_pos[2]-next_wp[2])**2)

        # Check if we've reached next waypoint within tolerance. If so increment where we plan from
        if self.next_wp_mode == 0:
            if ((dist_next_wp<self.visit_tol) and self.plan_ind_curr < len(self.global_plan_base)-2):
                self.plan_ind_curr += 1
        else:
            if ((dist_next_wp-self.dist_next_wp_prev)>self.dist_next_wp_prev_tol and self.plan_ind_curr < len(self.global_plan_base)-2):
                self.dist_next_wp_prev = 10000
                self.plan_ind_curr += 1
            else:
                self.dist_next_wp_prev = dist_next_wp

        # Send out global plan as current point to the next global_plan_len waypoints, clipped to end goal location
        plan_end_ind = min(self.plan_ind_curr+self.global_plan_len,len(self.global_plan_base)-1)

        # Build new plan starting from current state
        self.global_plan = Path()
        new_path_header = Header(stamp=rospy.get_rostime(),frame_id = "world")
        pose_list_readable = [curr_pos]
        pose_list = [PoseStamped(header = new_path_header, pose = Pose(position=Point( x = curr_pos[0],
                                                                                    y = curr_pos[1],
                                                                                    z = curr_pos[2])))]
        for node in self.global_plan_base[self.plan_ind_curr+1:plan_end_ind+1]:
            pose_new = PoseStamped(header = new_path_header, pose=Pose(position=Point(x = node[0],
                                                    y = node[1],
                                                    z = node[2])))
            pose_list.append(pose_new)
            pose_list_readable.append(node)
        self.global_plan.poses = pose_list
        self.global_plan.header = new_path_header

        self.glob_plan_pub.publish(self.global_plan)

        if self.verbose and self.report_counter%self.report_interval == 0:
            rospy.loginfo("Published global plan: {}".format(pose_list_readable))
        
        self.report_counter += 1


    def state_sub_callback(self,msg):
        # TODO: May need to consider thread safety
        self.state = msg
        if not self.received_state:
            self.received_state = True

    def master_node_state_sub_callback(self,msg):
        # TODO: May need to consider thread safety
        self.master_node_state = msg

if __name__ == '__main__' :
    try:
        # Initialize node
        rospy.init_node('fake_global_planner')
        global_planner = fake_global_planner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass