#!/usr/bin/env python

import math
import rospy
from snapstack_msgs.msg import Goal, State
from geometry_msgs.msg import PoseStamped, Vector3
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud, ChannelFloat32
from std_msgs.msg import Header
import random
import time

# Node : (x, y, z, occ_sts, cost)
# PointCloud : (x, y, z), ChannelFloat32 : occupancy info
# goal and state: (x, y, z)
# modes: STRAIGHT or DIAGONAL
# distance metrics: EUCLIDEAN or MANHATTAN
# width of the Bounding Box : BB_WIDTH
#
#
#
class globalPlanner:
    def __init__(self):
        #Constants
        self.DIAGONAL = 1
        self.STRAIGHT = 2
        self.BB_BOX_DIST = 2
        self.RESOLUTION = 0.5
        self.BB_WIDTH = int(self.BB_BOX_DIST/self.RESOLUTION)
        self.EUCLIDEAN_DIST = 1
        self.MANHATTAN_DIST = 0
        self.INFINITE_COST = 1000
        self.OCCUPIED = 1
        self.NOT_OCCUPIED = 0
        self.GRID_WIN_WIDTH = 3 
        self.stride_z = 4
        self.stride_y = 16
        self.stride_x = 64
        self.D_LIMIT = int(math.floor(self.BB_WIDTH/2))
        self.X_LIMIT = [0 , self.BB_WIDTH]
        self.Y_LIMIT = [-self.D_LIMIT,    self.D_LIMIT]
        self.Z_LIMIT = [-self.D_LIMIT,    self.D_LIMIT]
        self.COST_EMPTY_CELL = 1
        self.COST_OCCUPIED_CELL = 1000 #Obstacle
        self.COST_GOAL = 0
        self.COST_WALL = 100
        self.pub_rate = rospy.Rate(10)	 #10Hz

        #Variables
        self.header = Header()
        self.global_nodes_path = Path()
        self.nxt_move = self.STRAIGHT #Can be straight or diagonal
        self.nxt_move_num = 0 #Move number
        self.num_of_cells = 0 
        self.cur_goal = Goal()
        self.cur_state = [0,0,0]
        self.cur_node = []
        self.new_nodes = []
        self.occupancy_sts = []
        self.global_path = Path()
        self.path_nodes_list = []
        self.map_points = PointCloud()
        self.occupancy_sts = ChannelFloat32()
        self.occupancy_sts.name = 'grid occupancy'
        self.seq_cntr = 0
        self.stop_jps = 0
        self.cntr = 0
        self.start = 0
        self.end = 0
        self.debug_en = False
        self.grid = [0,0,0,0,0,0,0,0,0,   0,0,0,0,0,1,0,0,0,   0,0,0,0,0,0,0,0,0]
        self.sim_en = True
        self.sim_path_nodes_list =[]
      
        #Publishers
        self.global_path_pub = rospy.Publisher('global_plan', Path, queue_size=100)
        self.goal_pub = rospy.Publisher('/SQ01s/goal', Goal, queue_size=100)

        #Dictionaries
        self.coord_offset_dict = {
        0  : [0, -1, -1] ,
        1  : [1, -1, -1] ,
        2  : [2, -1, -1] ,
        3  : [0, 0, -1] ,
        4  : [1, 0, -1] ,
        5  : [2, 0, -1] ,
        6  : [0, 1, -1] ,
        7  : [1, 1, -1] ,
        8  : [2, 1, -1] ,
        9  : [0, -1, 0] ,
        10  : [1, -1, 0] ,
        11  : [2, -1, 0] ,
        12  : [0, 0, 0] ,
        13  : [1, 0, 0] ,
        14  : [2, 0, 0] ,
        15  : [0, 1, 0] ,
        16  : [1, 1, 0] ,
        17  : [2, 1, 0] ,
        18  : [0, -1, 1] ,
        19  : [1, -1, 1] ,
        20  : [2, -1, 1] ,
        21  : [0, 0, 1] ,
        22  : [1, 0, 1] ,
        23  : [2, 0, 1] ,
        24  : [0, 1, 1] ,
        25  : [1, 1, 1] ,
        26  : [2, 1, 1] }

        """offset_dict = {
          "forward" :           14  ,   #13 
          "forward_left":       11  ,   #10
          "forward_right" :     17  ,   #16    
          "forward_up" :        23  ,   #22
          "forward_down" :       5  ,   #4
          "forward_left_up":    20  ,   #19
          "forward_left_down" :  2  ,   #1
          "forward_right_up" :  26  ,   #25
          "forward_right_down" : 8  ,   #7
          "left" :               9  ,
          "right" :             15  ,
          "up" :                21  ,
          "down" :               3  ,
          "left_up" :           18  ,
          "left_down" :          0  ,
          "right_up" :          24  ,
          "right_down" :         6      
      }"""

    """#Read the goal and State"""

    def read_rviz_goal(self,msg):        
      self.cur_goal = [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]
      rospy.loginfo(self.cur_goal)
      if self.debug_en:
        rospy.loginfo("New goal!")  
      self.dist_to_goal = self.calc_dist_btw_nodes(self.cur_state, self.cur_goal,self.MANHATTAN_DIST)

    def read_state(self,msg):
      self.cntr += 1
      self.cur_state = [msg.pos.x,msg.pos.y, msg.pos.z]
      if self.debug_en:
        rospy.loginfo(self.cntr)
        rospy.loginfo(self.cur_state)
      self.start = time.time()

    """Generate the 3D Map from the point clouds containing occupancy and costs (5D array)
    
    ###Read the nulti-array and generate the 3D Map"""

    def read_map(self,pointClouds):
      self.map_points.points = pointClouds.points
      self.occupancy_sts.values = pointClouds.channels[0].values
      self.grid = pointClouds.channels[0].values
      rospy.loginfo(len(pointClouds.points))
      if len(pointClouds.points) == 27:        
        self.get_next_state()
      else:
        rospy.loginfo(len(pointClouds.points))
        print("length of grid received is not 27")

    """###Calculate the next state"""

    def get_next_state(self):
      self.path_nodes_list = [self.cur_state]
      nxt_state_off = self.select_move(self.grid)
      self.nxt_state = self.get_coordinates(nxt_state_off, self.cur_state)
      self.path_nodes_list.append(self.nxt_state)
      self.publish_global_plan()

    """###Select Move"""

    def select_move(self,grid):
      nxt_state = 12 #Same position
      offsets = [17,11,14,23,5,20,2,26,8,9,15,21,3,18,0,24,6] #Refer the offsets_dict
      for offset in offsets:
        if grid[offset] == self.NOT_OCCUPIED:
          self.nxt_move = self.straight_or_diag(offset)
          nxt_state = offset
          break
      return nxt_state

    """###Set mode to straight or diagonal"""
    def straight_or_diag(self,move_off):      #TODO--Add goal logic
      straight_moves = [13,14,9,15,21,3]
      if straight_moves.count(move_off) > 0:
        return self.STRAIGHT
      else:
        return self.DIAGONAL

    """###Convert offsets back to coordinates"""
    def get_coordinates(self,offset,cur_position):
      off_position = cur_position    
      coordinates_off = (self.coord_offset_dict[offset])
      cur_offset = [off*self.RESOLUTION for off in coordinates_off]
      off_position = [cur_position[i]+cur_offset[i] for i in range(0,3)]
      return off_position
  
    """#Publish the Path"""

    def publish_global_plan(self):
      self.seq_cntr += 1            
      self.header.seq = self.seq_cntr
      self.header.stamp = rospy.Time.now()
      self.header.frame_id = 'world'
      self.global_nodes_path = Path()	    
      pose_list = []
      i = 0
      print(self.cur_state)
      print(self.path_nodes_list)
      for path_node in self.path_nodes_list:
        node_pose = PoseStamped()       
        node_pose.pose.position.x = path_node[0]
        node_pose.pose.position.y = path_node[1]
        node_pose.pose.position.z = path_node[2]
        node_pose.pose.orientation.x = 0
        node_pose.pose.orientation.y = 0
        node_pose.pose.orientation.z = 0
        node_pose.pose.orientation.w = 1
        node_pose.header.stamp = rospy.Time.now()
        node_pose.header.seq = i
        node_pose.header.frame_id = 'world'
        pose_list.append(node_pose)
        i += 1
      self.global_nodes_path.header= self.header
      self.global_nodes_path.poses = pose_list
      self.global_path_pub.publish(self.global_nodes_path)
      self.end = time.time()
      if self.debug_en:
        rospy.loginfo(self.end -  self.start)

    """#Simulation of Goal"""
    def fill_dummy_path(self):
      self.sim_path_nodes_list = []
      cntr = self.seq_cntr
      for i in range(0,4):
        cntr += 1
        self.sim_path_nodes_list.append([0.05*cntr*i, 0.02*cntr*i, 0.01*cntr*i])
      if self.seq_cntr > 300:
          self.seq_cntr = 0 

    """#Publish the Goal"""
    def publish_sim_path(self):
      self.seq_cntr += 1            
      self.header.seq = self.seq_cntr
      self.header.stamp = rospy.Time.now()
      self.header.frame_id = 'world'    
      self.fill_dummy_path()
      i = 0
      self.sim_path_nodes_list = self.path_nodes_list
      for path_node in self.sim_path_nodes_list:
        i += 1
        if i==2:
          self.cur_goal.p.x = path_node[0]
          self.cur_goal.p.y = path_node[1]
          self.cur_goal.p.z = path_node[2]
          self.cur_goal.v.x = 2.5
          self.cur_goal.yaw = 0.1
          self.cur_goal.header = self.header    
      self.goal_pub.publish(self.cur_goal)

"""#Main module"""

if __name__ == '__main__':
    rospy.init_node('globalPlanner')
    try:
        globalPlanner_o = globalPlanner()
        rate = rospy.Rate(100)
#Subscribers        
        rospy.Subscriber("/SQ01s/state" , State, globalPlanner_o.read_state)
        rospy.Subscriber("grid_publisher" , PointCloud, globalPlanner_o.read_map)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, globalPlanner_o.read_rviz_goal)
        while (not rospy.is_shutdown()):
          globalPlanner_o.publish_sim_path()
          rate.sleep()
	#Infinite Loop
    except rospy.ROSInterruptException:  pass
