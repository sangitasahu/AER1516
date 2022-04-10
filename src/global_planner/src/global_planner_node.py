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

    """#Initialise the globalPlanner instance"""
    def __init__(self):

        """#Constants"""

        #Directions 
        self.DIAGONAL1 = 1
        self.DIAGONAL2 = 2
        self.STRAIGHT = 0
        #Map related
        self.BB_WIDTH = 2
        self.STOP_DISTANCE = 1.414
        self.RESOLUTION = 0.125
        self.X_LIMIT = [0 , 100]
        self.Y_LIMIT = [0,  100]
        self.Z_LIMIT = [0,  30]
        self.ALLOWED_POS_ERR = 2*math.sqrt(self.RESOLUTION)
        self.ALLOWED_GOAL_POS_ERR = 2*math.sqrt(self.RESOLUTION**3)
        #Distance metrics
        self.EUCLIDEAN_DIST = 1
        self.MANHATTAN_DIST = 0
        self.INIFINITY = 10000
        #Cost related
        self.COST_GOAL = 0
        self.COST_EMPTY_CELL = 1
        self.COST_OCCUPIED_CELL = 100 #Obstacle
        self.COST_WALL = self.INIFINITY  
        #Status flags
        self.OCCUPIED = 1
        self.NOT_OCCUPIED = 0
        #JPS related
        self.GRID_WIN_WIDTH = 3
        self.MAX_NUM_ITERATIONS = 100
        self.STRAIGHT_MOVES = [13,9,15,21,3] 
        self.DIAGONAL1_MOVES = [10,16,18,0,22,4,24,6]
        self.DIAGONAL2_MOVES = [19,1,25,7]               
        self.MOVES = self.STRAIGHT_MOVES  + self.DIAGONAL1_MOVES + self.DIAGONAL2_MOVES 
        
        """#Variables"""

        #Messages related
        self.header = Header()
        self.nxt_goal = Goal()
        self.global_path = Path()
        self.global_nodes_path = Path()
        self.map_points = PointCloud()
        self.occupancy_sts = ChannelFloat32()
        self.occupancy_sts.name = 'grid occupancy'
        
        #JPS related
        self.nxt_move = self.STRAIGHT #Can be straight or diagonal
        self.nxt_move_num = 0 #Move number
        self.num_of_cells = 0 
        self.cur_goal = [29,30,3]        
        self.proj_cur_goal = [0,0,0]
        self.cur_state = [0,0,0]
        self.path_nodes_list = []
        self.grid = [0,0,0,0,0,0,0,0,0,   0,0,0,0,0,1,0,0,0,   0,0,0,0,0,0,0,0,0]
        self.cur_state_in_grid = [0,0,0]
        self.nxt_suitable_moves = [13]
        self.allowed_lengths = [27, 125, 729]
        self.total_path_cost=0
        self.visited_nodes_list =[0,0,0]

        #For simulation only
        self.cntr = 0
        self.sim_en = True
        self.sim_path_nodes_list =[]
        
        #Control flags and counters
        self.seq_cntr = 0
        self.stop_jps = 0
        self.num_iteration = 0
        
        #Timing and debugging
        self.start = 0
        self.end = 0
        self.debug_en = False
      
        #Publishers related
        self.pub_rate = rospy.Rate(10)	 #10Hz
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
        26  : [2, 1, 1] ,
        27  : [0,-1,2],
        28  : [0, 1, 2],
        29  : [0,-1, -2],
        30  : [0,-1,-2],
        31  : [1,2,0],
        32  : [1,-2,0],
        33  : [0,-2,0],
        34  : [0,2,0],
        35  : [0,0,2],
        36  : [0,0,-2] }

        self.centre_dict = {27: 12, 125: 60, 729: 360}  #Allowed resolutions

        self.neighbor_offsets_dict = {
          13 : [10,11,16,17],
          21 : [18,27,24,28],
          3 : [0,29,6,31],
          15 : [16,31],
          9 : [10,32],
          10 : [13,14,9,33],
          16 : [13,14,15,34],
          18 : [9,33,21,35],
          0 : [9,33,3,36],
          24 : [15,34,21,35],
          6 : [15,34,3,36],
          22 : [13,14,21,35],
          4 : [13,14,3,36],
          19 : [13,14,9,33,21,35],
          1: [13,14,9,33,3,36],
          25 :[13,14,15,34,21,35],
          7 :[13,14,15,34,3,36],
          12 : [0,0] }

        self.suitable_moves_dict = {
          13 : [11,17],
          21 : [18,24],
          3 :  [0,6],
          15 : [16,24],
          9 :  [10,18],
          10 : [13,9],
          16 : [13,15],
          18 : [9,21],
          0 :  [9,3],
          24 : [15,21],
          6 :  [15,3],
          22 : [13,21],
          4 :  [13,3],
          19 : [13,9,21],
          1:   [13,9,3],
          25 : [13,15,21],
          7 :  [13,15,3] }

    """#Read the goal and current state of the drone"""

    def read_rviz_goal(self,msg):        
      self.cur_goal = [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]
      rospy.loginfo(self.cur_goal)
      if self.debug_en:
        rospy.loginfo("New goal!")  
      
    def read_state(self,msg):
      self.cntr += 1
      self.cur_state = [msg.pos.x,msg.pos.y, msg.pos.z]
      d_to_goal = self.calc_dist_btw_nodes(self.cur_state, self.cur_goal,self.EUCLIDEAN_DIST)
      #print("Cur distance to goal", d_to_goal,self.cur_state  )
      if d_to_goal < self.STOP_DISTANCE:
        self.stop_jps = 1
        print("Reached the goal")
      else:
        if self.debug_en:
          rospy.loginfo(self.cntr)
          rospy.loginfo(self.cur_state)      

    """Generate the 3D Map from the point clouds containing occupancy and costs (5D array)
    
    ###Read the nulti-array and generate the 3D Map"""

    def read_map(self,pointClouds):
      self.map_points.points = pointClouds.points
      self.occupancy_sts.values = pointClouds.channels[0].values
      #print(self.occupancy_sts.values)
      self.grid = pointClouds.channels[0].values
      self.num_of_cells = len(pointClouds.points)
      if self.stop_jps == 0:        
        self.start = time.time()
        if self.allowed_lengths.count(len(pointClouds.points)) > 0: 
          start_state = pointClouds.points[self.centre_dict[self.num_of_cells]]
          self.cur_state_in_grid = [start_state.x, start_state.y, start_state.z]    
          self.get_next_state()          
        else:         
          print("length of grid received is ", len(pointClouds.points), "which is invalid !")
    
    """###Update path nodes list"""
    def update_path_nodes_list(self,cur_state):
      start_appending = False
      updated_nodes_list = []
      for node in self.path_nodes_list:
        if start_appending:
          updated_nodes_list.append(node)
        if self.is_same_node(node, cur_state):
          start_appending = True
      return updated_nodes_list

    """###Get projected goal for current JPS iteration"""
    def get_projected_goal(self):
      min_d_goal = self.INIFINITY 
      for i in range(0, self.num_of_cells):
        cur_point = [self.map_points.points[i].x, self.map_points.points[i].y, self.map_points.points[i].z]
        if self.occupancy_sts.values[i] == self.NOT_OCCUPIED:
          d_goal = self.calc_dist_btw_nodes(cur_point, self.cur_goal,self.EUCLIDEAN_DIST)
          if d_goal < min_d_goal:
            min_d_goal = d_goal
            self.proj_cur_goal = cur_point
      #print("Cur projected goal pt ", self.proj_cur_goal)
    

    """###Calculate distance between nodes"""

    def calc_dist_btw_nodes(self,node1, node2, typ):
      if typ == self.MANHATTAN_DIST:
        dist = abs(node2[0] -node1[0]) + abs(node2[1] - node1[1]) + abs(node2[2] - node1[2])
      else:
        dist = math.sqrt((abs(node2[0] -node1[0])**2) + (abs(node2[1] - node1[1])**2) + (abs(node2[2] - node1[2])**2))
      return dist


    """###Calculate the next state"""

    def get_next_state(self):
      self.path_nodes_list = []
      #print("Searching with JPS...")
      #print("Current state = ", self.cur_state_in_grid)
      self.goal_reached = False
      self.total_path_cost += self.COST_EMPTY_CELL
      self.get_projected_goal()
      next_states, path_cost, sts = self.get_jps_successors()
      print("Path valid = ", self.is_path_valid(next_states))
      if sts:
        self.total_path_cost += path_cost
        #print("next jumps =", next_states)
        for next_state in next_states:
          self.path_nodes_list.append(next_state)
      #print("Publish ", self.cur_state_in_grid,self.path_nodes_list)
      self.publish_global_plan()
      #If end goal is reached
      if self.has_reached_goal(next_states[-1],self.cur_goal):
          self.goal_reached = True
          print("Reached end goal!")
          self.stop_jps = 1
          self.visited_nodes_list = []

    """###Select Move"""
    """offset_dict = {
          "forward" :           13  ,    
          "forward_left":       10  ,   
          "forward_right" :     16  ,       
          "forward_up" :        22  ,   
          "forward_down" :       4  ,   
          "forward_left_up":    19  ,   
          "forward_left_down" :  1  ,   
          "forward_right_up" :  25  ,   
          "forward_right_down" : 7  ,   
          "left" :               9  ,
          "right" :             15  ,
          "up" :                21  ,
          "down" :               3  ,
          "left_up" :           18  ,
          "left_down" :          0  ,
          "right_up" :          24  ,
          "right_down" :         6      
      }"""
    def select_moves(self, cur_state):
      min_dist = self.INIFINITY
      best_move = 12
      for move in self.MOVES:
        next_node = self.get_coordinates(move, cur_state)
        if self.is_reachable(next_node) == True and self.has_visited(next_node) == False:
          dist = self.calc_dist_btw_nodes(next_node, self.proj_cur_goal, self.EUCLIDEAN_DIST)
          if dist < min_dist:
            min_dist = dist
            best_move = move
      #print("Best move =", best_move)
      return best_move

    """###Calculate jps jump nodes"""
    def get_jps_successors(self):
      possible_paths_dict ={}     
      cur_path_cost = 0
      cur_path = []
      self.num_iteration = 0
      cur_grid_node = self.cur_state_in_grid
      paths = {}
      costs = {}
      num_of_paths_to_check = 1
      num_of_paths_checked = 0
      all_paths_analysed = False

      self.reached_cur_goal = False
      move_num = 0
      old_move = 12
      while num_of_paths_to_check - num_of_paths_checked > 0 :        
        self.nxt_move_num = self.select_moves(cur_grid_node)
        nxt_state = self.get_coordinates(self.nxt_move_num, self.cur_state_in_grid)            
        #print("Next state = ", nxt_state)
        if self.is_reachable(nxt_state):          
          next_nodes , next_costs = self.get_neighbours(self.nxt_move_num, cur_grid_node)
          #print("Next nodes = ", next_nodes)
          if self.reached_cur_goal == False:
            if len(next_nodes)>0:
              #print("Number of forced neighbours =", len(next_nodes)-1)
              for i in range(0,len(next_nodes)):
                if i==0:
                  cur_grid_node = next_nodes[i]
                  cur_path_cost += next_costs[i]
                  if old_move == self.nxt_move_num:                  
                    cur_path[-1] = cur_grid_node
                  else:
                    cur_path.append(cur_grid_node)               
                else:
                  cost_of_cur_grid = self.calc_node_cost(next_nodes[i]) 
                  num_of_paths_to_check += 1                 
                  paths[num_of_paths_to_check] = cur_path.append(next_nodes[i])    
                  costs[num_of_paths_to_check] = cur_path_cost+cost_of_cur_grid
              #print("Next cur state in grid = ", cur_grid_node )
              self.visited_nodes_list.append(cur_grid_node)  #Can grow large
              self.num_iteration += 1
              if self.num_iteration == 25: #self.MAX_NUM_ITERATIONS - 1:
                #print("Max iterations reached !")                                
                nxt_paths = list(paths.keys())
                nxt_costs = list(costs.keys())
                old_move = 12
                #print("Paths n costs =",paths)               
                self.num_iteration = 0
                if len(nxt_paths) > 0:
                  #print("next paths =", nxt_paths,paths[nxt_paths[0]])
                  num_of_paths_checked += 1
                  cur_path = paths[nxt_paths[0]] 
                  #print("Cur path =", cur_path)
                  cur_grid_node = cur_path[-1]
                  cur_path_cost = costs[nxt_costs[0]]
                  #Delete traversed path         
                  del paths[nxt_paths[0]]
                  del costs[nxt_costs[0]]
                else:
                  break
          else:
            self.num_iteration = 0
            num_of_paths_checked += 1
            cur_path.append(self.proj_cur_goal) 
            possible_paths_dict[cur_path_cost] = cur_path
            print("Projected goal reached!")

            #If no more paths to check
            if num_of_paths_to_check == num_of_paths_checked:
              all_paths_analysed = True
              break
            else:
              nxt_paths = list(paths.keys())
              nxt_costs = list(costs.keys())
              old_move = 12
              #print("Paths n costs =",paths)              
              self.num_iteration = 0
              if len(nxt_paths) > 0:
                #print("next paths =", nxt_paths,paths[nxt_paths[0]])
                num_of_paths_checked += 1
                cur_path = paths[nxt_paths[0]] 
                #print("Cur path =", cur_path)
                cur_grid_node = cur_path[-1]
                cur_path_cost = costs[nxt_costs[0]] 
                #Delete traversed path        
                del paths[nxt_paths[0]]
                del costs[nxt_costs[0]]
                break
        old_move = self.nxt_move_num
      
      if all_paths_analysed:
        min_cost = min(list(possible_paths_dict.keys()))
        best_path = possible_paths_dict[min_cost]
      else:
        min_cost = 0
        best_path = [self.cur_state_in_grid]

      return best_path, min_cost, all_paths_analysed
    
    """###Check path is valid"""
    def is_path_valid(self, path):
      for node in path:
        print("Node = ", node, "Valid =",self.is_reachable(node) )        
        if not self.is_reachable(node):
          return False
      return True

    """###Calculate cost of a node"""
    def calc_node_cost(self,node):
      if self.is_same_node(node,self.proj_cur_goal):
        return self.COST_GOAL
      elif (node[0] <= (self.X_LIMIT[0] + self.ALLOWED_POS_ERR)) or (node[0] >= (self.X_LIMIT[1] - self.ALLOWED_POS_ERR)): 
        return self.COST_WALL
      elif (node[1] <= (self.Y_LIMIT[0] + self.ALLOWED_POS_ERR)) or (node[1] >= (self.Y_LIMIT[1] - self.ALLOWED_POS_ERR)): 
        return self.COST_WALL
      elif (node[2] <= (self.Z_LIMIT[0] + self.ALLOWED_POS_ERR)) or (node[2] >= (self.Z_LIMIT[1] - self.ALLOWED_POS_ERR)):  
        return self.COST_WALL      
      elif self.is_occupied(node) ==  self.OCCUPIED:
        return self.COST_OCCUPIED_CELL
      else:
        return self.COST_EMPTY_CELL

    """###Get neighbours for a move"""
    def get_neighbours(self,nxt_move_num,cur_state):
      next_neighbour = []      
      neigh_nodes = []
      next_costs = []
      neigh_nodes_sts =[]
      
      #Evaluate neighbours
      neighbours_to_chk = self.neighbor_offsets_dict[nxt_move_num]
      
      num_of_neighbors = len(neighbours_to_chk)
      for neigh_node_idx in neighbours_to_chk:
        neigh_node_coord = self.get_coordinates(neigh_node_idx, cur_state)
        neigh_nodes.append(neigh_node_coord)
        neigh_nodes_sts.append(self.is_reachable(neigh_node_coord))
        if self.has_reached_goal(neigh_node_coord,self.proj_cur_goal):
          print("Reached current goal!")
          self.reached_cur_goal = True
          next_neighbour.append(neigh_node_coord)
          return next_neighbour , self.COST_GOAL

      #Add next state
      next_state = self.get_coordinates(nxt_move_num, cur_state)  
      next_neighbour.append(next_state)
      next_costs.append(self.calc_node_cost(next_state)+ self.calc_dist_btw_nodes(cur_state,next_state,self.EUCLIDEAN_DIST))

      #Check for forced neighbours
      if num_of_neighbors == 2:
        if neigh_nodes_sts[1] == True and neigh_nodes_sts[0] == False :
          next_neighbour.append(neigh_nodes[1])
          next_costs.append(self.calc_node_cost(neigh_nodes[1]) + self.calc_dist_btw_nodes(cur_state,neigh_nodes[1],self.EUCLIDEAN_DIST))
      elif num_of_neighbors == 4:
        if neigh_nodes_sts[1] == True and neigh_nodes_sts[0] == False :
          next_neighbour.append(neigh_nodes[1])
          next_costs.append(self.calc_node_cost(neigh_nodes[1]) + self.calc_dist_btw_nodes(cur_state,neigh_nodes[1],self.EUCLIDEAN_DIST))
        if neigh_nodes_sts[3] == True and neigh_nodes_sts[2] == False :
          next_neighbour.append(neigh_nodes[3])
          next_costs.append(self.calc_node_cost(neigh_nodes[3])+ self.calc_dist_btw_nodes(cur_state,neigh_nodes[3],self.EUCLIDEAN_DIST))
      else: #if six neighbours
        if neigh_nodes_sts[1] == True and neigh_nodes_sts[0] == False :
          next_neighbour.append(neigh_nodes[1])
          next_costs.append(self.calc_node_cost(neigh_nodes[1]) + self.calc_dist_btw_nodes(cur_state,neigh_nodes[1],self.EUCLIDEAN_DIST))
        if neigh_nodes_sts[3] == True and neigh_nodes_sts[2] == False :
          next_neighbour.append(neigh_nodes[3])
          next_costs.append(self.calc_node_cost(neigh_nodes[3])+ self.calc_dist_btw_nodes(cur_state,neigh_nodes[3],self.EUCLIDEAN_DIST))
        if neigh_nodes_sts[5] == True and neigh_nodes_sts[4] == False :
          next_neighbour.append(neigh_nodes[5])
          next_costs.append(self.calc_node_cost(neigh_nodes[3])+ self.calc_dist_btw_nodes(cur_state,neigh_nodes[3],self.EUCLIDEAN_DIST))
      
      return next_neighbour, next_costs

    """###Check whether goal is reachable"""
    def has_reached_goal(self, node1, node2):
      if self.calc_dist_btw_nodes(node1, node2, self.EUCLIDEAN_DIST) < self.ALLOWED_GOAL_POS_ERR :
        return True
      return False

    """####Check whether same node"""
    def is_same_node(self,node1, node2):
      if node1[0]-node2[0] < self.ALLOWED_POS_ERR and node1[1]-node2[1] < self.ALLOWED_POS_ERR and node1[2]-node2[2] < self.ALLOWED_POS_ERR:
        return True
      return False

    """###Check whether a node is already visited"""
    def has_visited(self,new_node):
      for node in self.visited_nodes_list:
        if new_node == node:
          return True
      return False

    """###Check whether reachable or not"""
    def is_reachable(self,node):
      reachable_sts = False
      if self.is_occupied(node) == self.NOT_OCCUPIED:
        if (self.Z_LIMIT[0] < node[2] < self.Z_LIMIT[1]) and (self.Y_LIMIT[0] < node[1] < self.Y_LIMIT[1]) and (self.X_LIMIT[0] < node[0] < self.X_LIMIT[1]):
          reachable_sts = True
      return reachable_sts

    """###Check Occupancy status"""
    def is_occupied(self,node):
      node_idx = 0
      occ_sts = self.OCCUPIED #For nodes not in the map, the status is OCCUPIED
      for point in self.map_points.points:
        if self.is_same_node([point.x, point.y, point.z], node):
          occ_sts = self.occupancy_sts.values[node_idx] 
          print("Node idx",node_idx, node, "Occ = ", occ_sts)
          break
        node_idx += 1
      return occ_sts

    """###Set mode to straight or diagonal"""
    def straight_or_diag(self,num):       
      if self.STRAIGHT_MOVES.count(num) > 0:
        nxt_move = self.STRAIGHT
      elif self.DIAGONAL1_MOVES.count(num) > 0:
        nxt_move = self.DIAGONAL1
      elif self.DIAGONAL2_MOVES.count(num) > 0:
        nxt_move = self.DIAGONAL2
      return nxt_move
      
    """###Convert offsets back to coordinates"""
    def get_coordinates(self,offset,cur_position):
      off_position = cur_position    
      coordinates_off = (self.coord_offset_dict[offset])
      cur_offset = [off*self.RESOLUTION for off in coordinates_off]
      #print("Off =",offset, "Pos =", cur_position)
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
      #print(self.path_nodes_list)
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
          self.nxt_goal.p.x = path_node[0]
          self.nxt_goal.p.y = path_node[1]
          self.nxt_goal.p.z = path_node[2]
          self.nxt_goal.v.x = 2.5
          self.nxt_goal.yaw = 0.1
          self.nxt_goal.header = self.header    
      self.goal_pub.publish(self.nxt_goal)

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
