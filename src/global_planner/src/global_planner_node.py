#!/usr/bin/env python

import math
import rospy
from snapstack_msgs.msg import Goal, State
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud, ChannelFloat32
from std_msgs.msg import Header
import random
import time
import copy

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

        #Map related
        self.BB_WIDTH = 18  #Change for different map sizes
        self.BB_WIDTH_Z = 3
        self.STOP_DISTANCE = 1
        self.RESOLUTION = 0.5
        self.X_LIMIT = [0 , 100]
        self.Y_LIMIT = [0,  100]
        self.Z_LIMIT = [0,  10]
        self.ALLOWED_POS_ERR = 2*math.sqrt(self.RESOLUTION)
        self.ALLOWED_GOAL_POS_ERR = 2*math.sqrt(self.RESOLUTION**3)
        self.NUM_OF_CELLS = int(((self.BB_WIDTH/self.RESOLUTION)+1)**2) * int((self.BB_WIDTH_Z/self.RESOLUTION)+1)
        self.ORIGIN_CELL_NUM = int(self.NUM_OF_CELLS/2) 
        #Distance metrics
        self.EUCLIDEAN_DIST = 1
        self.MANHATTAN_DIST = 0
        self.INFINITY = 10000
        #Cost related
        self.COST_GOAL = 0
        self.COST_EMPTY_CELL = 1
        self.COST_OCCUPIED_CELL = 100 #Obstacle
        self.COST_WALL = self.INFINITY  
        self.COST_UNKNOWN = 0
        #Status flags
        self.OCCUPIED = 1
        self.NOT_OCCUPIED = 0
        self.UNKNOWN = -1
        self.PROJ_GOAL_REACHED = 1
        self.MAX_ITERATIONS_REACHED = 2
        self.MOVE_NOT_POSSIBLE = 3
        self.COMPLETED = 1
        self.RUNNING = 0
        #JPS related
        self.MAX_NUM_ITERATIONS = 100
        self.MAX_PATHS_TO_CHECK = 5
        #Moves      
        self.STRAIGHT_MOVES_1 = [13,9,15,21,3] 
        self.DIAGONAL1_MOVES_1 = [10,16,18,0,22,4,24,6]
        self.DIAGONAL2_MOVES_1 = [19,1,25,7]                
        self.MOVES = self.STRAIGHT_MOVES_1  + self.DIAGONAL1_MOVES_1 #+ self.DIAGONAL2_MOVES_1 
         
        """#Variables"""

        #Messages related
        self.header = Header()
        self.nxt_goal = Goal()
        self.global_nodes_path = Path()
        self.map_points = PointCloud()
        self.occupancy_sts = ChannelFloat32()
        self.new_map_points = PointCloud()
        self.new_occupancy_sts = ChannelFloat32()
        self.new_occupancy_sts.name = 'grid occupancy'
        
        #JPS related 
        self.nxt_move_num = 0 #Move number
        self.num_of_cells = 0
        self.new_goal =[0,0,0] 
        self.cur_goal = [0,0,0]
        self.new_state = [0,0,0]        
        self.proj_cur_goal = [0,0,0]
        self.cur_state = [0,0,0]
        self.path_nodes_list = []
        self.cur_state_in_grid = [0,0,0]
        self.new_cur_state_in_grid = [0,0,0]
        self.visited_nodes_list =[0,0,0]  
        self.neighbour_jump_offsets = []
        self.paths = {}
        self.costs = {}
        self.num_of_paths_to_check = 1
        self.num_of_paths_checked = 0
        self.all_paths_analysed = False
        self.run_num = 0
        self.reached_cur_goal = False
        self.old_move = 12

        #Control flags and counters
        self.seq_cntr = 0
        self.num_iteration = 0
        self.init_cond_flag = self.PROJ_GOAL_REACHED
        self.jps_run_sts = self.COMPLETED
        
        #Timing and debugging
        self.start = 0
        self.end = 0
        self.debug_en = False
      
        #Publishers related
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
        30  : [0, 1,-2],
        31  : [1,2,0],
        32  : [1,-2,0],
        33  : [0,-2,0],
        34  : [0,2,0],
        35  : [0,0,2],
        36  : [0,0,-2],
        37  : [0,-2,1],
        38  : [0,-2,-1] ,
        39  : [0,2,1],
        40  : [0,2,-1] ,
        41  :  [1,0,2],
        42  : [1,0,-2]}

        self.neighbor_offsets_dict = {
          13 : ([10,11,16,17],[14,14]),
          21 : ([18,27,24,28,35],[35,35]),
          3 : ([0,29,6,30],[36,36]),
          15 : ([16,31],[34,34]),
          9 : ([10,32],[33,33]),
          10 : ([13,14,9,33],[11,32]),
          16 : ([13,14,15,34],[17,31]),
          18 : ([9,33,21,35],[37,27]),
          0 : ([9,33,3,36],[38,29]),
          24 : ([15,34,21,35],[39,28]),
          6 : ([15,34,3,36],[40,30]),
          22 : ([13,14,21,35],[23,41]),
          4 : ([13,14,3,36],[5,42]),
          19 : ([13,14,9,33,21,35],[14,14,14]),
          1: ([13,14,9,33,3,36],[14,14,14]),
          25 :([13,14,15,34,21,35],[14,14,14]),
          7 :([13,14,15,34,3,36],[14,14,14]),
          12 : ([12,12],[12]) }

    """#Read the goal location for the drone"""
    def read_goal(self, msg):
      if not([msg.point.x, msg.point.y, msg.point.z] == self.new_goal):
        self.new_goal = [msg.point.x, msg.point.y, msg.point.z]         
        self.jps_run_sts = self.COMPLETED
        self.path_nodes_list = []
        self.visited_nodes_list =[0,0,0]
        print("New goal received : ", self.new_goal)

    """#Read the current state of the drone"""      
    def read_state(self,msg):
      self.new_state = [msg.pos.x,msg.pos.y, msg.pos.z]
      d_to_goal = self.calc_dist_btw_nodes(self.new_state, self.cur_goal,self.EUCLIDEAN_DIST)
      if d_to_goal < self.STOP_DISTANCE and self.jps_run_sts == self.RUNNING:
        self.jps_run_sts = self.COMPLETED
        print("Reached the end goal.......")     

    """Generate the 3D Map from the point clouds containing occupancy info"""
    def read_map(self,pointClouds):
      self.new_map_points.points = pointClouds.points
      self.new_occupancy_sts.values = pointClouds.channels[0].values
      self.num_of_cells = len(self.new_occupancy_sts.values) 
      print("New map!")     
      if self.jps_run_sts == self.COMPLETED:         
        if len(pointClouds.points) == self.NUM_OF_CELLS: 
          start_state = pointClouds.points[self.ORIGIN_CELL_NUM]
          self.new_cur_state_in_grid = [start_state.x, start_state.y, start_state.z]             
        else:         
          print("Length of grid received is ", len(pointClouds.points), "which is invalid !")
          print("Correctly set the BB_WIDTH and RESOLUTION variables!")
      else:
        print("Length of grid received is ", len(pointClouds.points), self.NUM_OF_CELLS, self.ORIGIN_CELL_NUM)

    """###Calculate the next state"""
    def get_next_state(self):
      if self.jps_run_sts == self.COMPLETED and len(self.new_occupancy_sts.values) == self.NUM_OF_CELLS:
        start_time_jps = time.time()

        #Store a copy of the variables
        self.cur_goal = copy.deepcopy(self.new_goal)
        self.cur_state = copy.deepcopy(self.new_state)
        self.cur_state_in_grid = copy.deepcopy(self.new_cur_state_in_grid)
        self.map_points = copy.deepcopy(self.new_map_points)
        self.occupancy_sts = copy.deepcopy(self.new_occupancy_sts)        
        print("Initial state :", self.cur_state, self.cur_state_in_grid, self.cur_goal, self.proj_cur_goal)
        self.test()
        if self.debug_en :
          print("JPS started searching...", self.cur_state_in_grid)
        #Initialise
        self.path_nodes_list = [self.cur_state]
        self.goal_reached = False
        #Goal is behind the drone
        if self.goal_is_behind(self.cur_state_in_grid,self.cur_goal):
          self.path_nodes_list.append(self.cur_goal)
          print("Goal is behind the drone!")
        else:
          #Get the projected goals in unknown space and known free space
          self.get_projected_goal()
          #Calculate JPS path
          next_states, path_cost, sts = self.get_jps_successors()          
          val_sts, last_index = self.is_path_valid(next_states)
          if self.debug_en == False:
            print("Final path found : ", next_states)
          if sts==1 and val_sts==0:
            for node in next_states:
              self.path_nodes_list.append(node)
          elif sts==1 and val_sts ==1:
            for i in range(0,last_index):
              self.path_nodes_list.append(next_states[i])
        #Publish the global plan
        self.publish_global_plan()
        #If end goal is reached
        if self.has_reached_goal(next_states[-1],self.cur_goal):
            self.goal_reached = True
            print("Reached end goal!")
            self.jps_run_sts = self.COMPLETED
            self.visited_nodes_list = []
        end_time_jps = time.time()
        print("Time taken by JPS is ", end_time_jps - start_time_jps)
      else:
        print("Last JPS still running....")

    """###Get projected goal for current JPS iteration"""
    def get_projected_goal(self):
      self.start_p = time.time()
      min_d_goal = self.INFINITY
      min_d_goal_unk = self.INFINITY
      if self.goal_within_bbx(self.cur_goal, self.cur_state_in_grid) :
        self.proj_cur_goal = self.cur_goal
        self.unk_cur_goal = None
        print("Goal within the bounding box!")
      else:
        for i in range(0, self.num_of_cells):
          cur_point = [self.map_points.points[i].x, self.map_points.points[i].y, self.map_points.points[i].z]
          if self.occupancy_sts.values[i] == self.NOT_OCCUPIED:
            d_goal = self.calc_dist_btw_nodes(cur_point, self.cur_goal,self.EUCLIDEAN_DIST)
            if d_goal < min_d_goal:
              min_d_goal = d_goal
              self.proj_cur_goal = cur_point
          elif self.occupancy_sts.values[i] == self.UNKNOWN:
            d_goal_unk = self.calc_dist_btw_nodes(cur_point, self.cur_goal,self.EUCLIDEAN_DIST)
            if d_goal_unk < min_d_goal_unk:
              min_d_goal_unk = d_goal_unk
              self.unk_cur_goal = cur_point
      self.end_p = time.time()
      print("Time to read through the map : ", self.end_p - self.start_p)

    """###Calculate jps jump nodes"""
    def get_jps_successors(self):
      if self.debug_en:
        print("Current Positions = ", self.cur_state_in_grid, self.cur_state)
      self.visited_nodes_list = []
      self.num_iteration = 0      
      self.paths = {}
      self.costs = {}
      self.num_of_paths_to_check = 1
      self.num_of_paths_checked = 0
      self.all_paths_analysed = False
      self.run_num = self.num_of_paths_to_check
      self.reached_cur_goal = False
      self.old_move = 12
      cur_grid_node = self.cur_state_in_grid
      old_cur_grid_node = cur_grid_node
      possible_paths_dict ={}     
      cur_path_cost = 0
      cur_path = []      
      
      while self.num_of_paths_to_check - self.num_of_paths_checked > 0 :
        if self.debug_en : 
          print("----------------Run no:", self.run_num)       
        self.nxt_move_num = self.select_moves(cur_grid_node)
        nxt_state = self.get_coordinates(self.nxt_move_num, cur_grid_node)            
        if self.debug_en :
          print("Next state = ", nxt_state,self.is_reachable(nxt_state), self.is_occupied(nxt_state))
          print("Move_num = ", self.nxt_move_num)
        if self.nxt_move_num == 12:
          break 
        if self.is_reachable(nxt_state):          
          next_nodes , next_costs = self.get_neighbours(self.nxt_move_num, cur_grid_node)
          if self.debug_en :
            print("Next nodes = ", next_nodes,self.reached_cur_goal)
            
          if self.reached_cur_goal == False:
            if len(next_nodes)>0:
              if self.debug_en == False and self.num_of_paths_to_check ==1:
                print("Number of forced neighbours =", len(next_nodes)-1,nxt_state, self.nxt_move_num, next_nodes)
              for i in range(0,len(next_nodes)):
                if i==0:
                  old_cur_grid_node = cur_grid_node
                  cur_grid_node = next_nodes[i]
                  cur_path_cost += next_costs[i]
                  if self.old_move == self.nxt_move_num and self.old_move != 12 and len(next_nodes)==1:                  
                    cur_path[-1] = cur_grid_node
                  else:
                    cur_path.append(cur_grid_node)                               
                else:
                  cost_of_cur_grid = self.calc_node_cost(next_nodes[i]) 
                  self.num_of_paths_to_check += 1 
                  updated_cur_path = list(cur_path) 
                  intermediate_node = self.get_node(old_cur_grid_node,self.neighbour_jump_offsets[i-1])
                  if self.is_reachable(intermediate_node):
                    updated_cur_path.append(intermediate_node)
                    updated_cur_path.append(next_nodes[i])                
                    self.paths[self.num_of_paths_to_check] = updated_cur_path  
                    self.costs[self.num_of_paths_to_check] = cur_path_cost+cost_of_cur_grid
              if self.debug_en:
                print(cur_path, updated_cur_path)
                print("Paths N Costs" , self.paths, self.costs)
                print("Next cur state in grid = ", cur_grid_node )
              self.visited_nodes_list.append(cur_grid_node)  #Can grow large
              self.num_iteration += 1
              if self.num_iteration == self.MAX_NUM_ITERATIONS - 1:
                if self.debug_en :
                  print("Max iterations reached!")
                  break
                self.init_cond_flag = self.MAX_ITERATIONS_REACHED
                cur_path, cur_grid_node, cur_path_cost,break_sts  = self.init_nxt_search()
                if break_sts:
                  break
          else:
            self.init_cond_flag = self.PROJ_GOAL_REACHED
            print("Path =", cur_path)
            if self.old_move == self.nxt_move_num and self.old_move != 12:                  
              cur_path[-1] = self.proj_cur_goal
            else:
              cur_path.append(self.proj_cur_goal) 
            possible_paths_dict[cur_path_cost] = list(cur_path)
            
            if self.debug_en:
              print("Projected goal reached!",self.cur_state,self.proj_cur_goal,cur_path_cost,cur_path)
            cur_path, cur_grid_node, cur_path_cost,break_sts = self.init_nxt_search()
            if break_sts:
              break
        else:
          if self.debug_en:
            print("No path possible!")
          cur_path, cur_grid_node, cur_path_cost, break_sts  = self.init_nxt_search()
          if break_sts:
              break
        self.old_move = self.nxt_move_num
      
      if self.debug_en == False:
        print("Posiible paths =",possible_paths_dict)

      #Chose the best path
      path_costs = list(possible_paths_dict.keys())
      if self.all_paths_analysed and len(path_costs)>0:        
        min_cost = min(path_costs)
        best_path = possible_paths_dict[min_cost]
      else:
        min_cost = 0
        best_path = [self.cur_state_in_grid]
      
      if self.debug_en :
        print("All paths analysed =",self.all_paths_analysed)
        print("Best Path = ", best_path, "Best cost =", min_cost)

      return best_path, min_cost, self.all_paths_analysed
    
    """###Select the Best Move"""
    def select_moves(self, cur_state):
      min_dist = self.INFINITY
      best_move = 12
      for move in self.MOVES:
        next_node = self.get_coordinates(move, cur_state)
        if self.debug_en == False and (cur_state == [3.0, 3.0, 3.5]):
          print(cur_state,next_node,self.is_reachable(next_node),self.has_visited(next_node),format(self.calc_dist_btw_nodes(next_node, self.proj_cur_goal, self.EUCLIDEAN_DIST), ".2f"),self.is_occupied(next_node),self.is_in_unknown(next_node) )
        if self.is_reachable(next_node) == True and self.has_visited(next_node) == False:
          dist = self.calc_dist_btw_nodes(next_node, self.proj_cur_goal, self.EUCLIDEAN_DIST)
          if dist < min_dist:
            min_dist = dist
            best_move = move

      if self.debug_en == False and (cur_state == [3.0, 3.0, 3.5]): 
        print("Best move =", best_move)
      return best_move
    
    """###Get neighbours for a move"""
    def get_neighbours(self,nxt_move_num,cur_state):
      next_neighbours = []      
      neigh_nodes = []
      next_costs = []
      neigh_nodes_sts =[]
      self.neighbour_jump_offsets = []
      
      #Evaluate neighbours
      neighbours_to_chk = self.neighbor_offsets_dict[nxt_move_num][0]
      neighbours_jump_offsets = self.neighbor_offsets_dict[nxt_move_num][1]
      if self.debug_en == False and cur_state == [3.0, 3.0, 3.5]:
        print("Neigh to chk",neighbours_to_chk )
      num_of_neighbors = len(neighbours_to_chk)
      for neigh_node_idx in neighbours_to_chk:
        neigh_node_coord = self.get_coordinates(neigh_node_idx, cur_state)
        if self.debug_en == False and cur_state == [3.0, 3.0, 3.5]:
          print(nxt_move_num,neigh_node_idx,cur_state,neigh_node_coord,self.has_reached_goal(neigh_node_coord,self.proj_cur_goal))
        neigh_nodes.append(neigh_node_coord)
        neigh_nodes_sts.append(self.is_occupied(neigh_node_coord))
        if self.has_reached_goal(neigh_node_coord,self.proj_cur_goal):
          self.reached_cur_goal = True
          next_neighbours.append(neigh_node_coord)
          return next_neighbours , self.COST_GOAL

      #Add next state
      next_state = self.get_coordinates(nxt_move_num, cur_state)  
      next_neighbours.append(next_state)
      next_costs.append(self.calc_node_cost(next_state)+ self.calc_dist_btw_nodes(cur_state,next_state,self.EUCLIDEAN_DIST))

      #Check for forced neighbours
      if num_of_neighbors == 2:
        if neigh_nodes_sts[0] == self.OCCUPIED and neigh_nodes_sts[1] == self.NOT_OCCUPIED :
          next_neighbours.append(neigh_nodes[1])
          self.neighbour_jump_offsets.append(neighbours_jump_offsets[0])
          next_costs.append(self.calc_node_cost(neigh_nodes[1]) + self.calc_dist_btw_nodes(cur_state,neigh_nodes[1],self.EUCLIDEAN_DIST))
      elif num_of_neighbors == 4:
        if neigh_nodes_sts[0] == self.OCCUPIED and neigh_nodes_sts[1] == self.NOT_OCCUPIED :
          next_neighbours.append(neigh_nodes[1])
          self.neighbour_jump_offsets.append(neighbours_jump_offsets[0])
          next_costs.append(self.calc_node_cost(neigh_nodes[1]) + self.calc_dist_btw_nodes(cur_state,neigh_nodes[1],self.EUCLIDEAN_DIST))
        if neigh_nodes_sts[2] == self.OCCUPIED and neigh_nodes_sts[3] == self.NOT_OCCUPIED :
          next_neighbours.append(neigh_nodes[3])
          self.neighbour_jump_offsets.append(neighbours_jump_offsets[1])
          next_costs.append(self.calc_node_cost(neigh_nodes[3])+ self.calc_dist_btw_nodes(cur_state,neigh_nodes[3],self.EUCLIDEAN_DIST))
      else: #if six neighbours
        if (neigh_nodes_sts[0] == self.OCCUPIED) and (neigh_nodes_sts[1] == self.NOT_OCCUPIED) :
          next_neighbours.append(neigh_nodes[1])
          self.neighbour_jump_offsets.append(neighbours_jump_offsets[0])
          next_costs.append(self.calc_node_cost(neigh_nodes[1]) + self.calc_dist_btw_nodes(cur_state,neigh_nodes[1],self.EUCLIDEAN_DIST))
        if (neigh_nodes_sts[2] == self.OCCUPIED) and (neigh_nodes_sts[3] == self.NOT_OCCUPIED) :
          next_neighbours.append(neigh_nodes[3])
          self.neighbour_jump_offsets.append(neighbours_jump_offsets[1])
          next_costs.append(self.calc_node_cost(neigh_nodes[3])+ self.calc_dist_btw_nodes(cur_state,neigh_nodes[3],self.EUCLIDEAN_DIST))
        if (neigh_nodes_sts[4] == self.OCCUPIED )and (neigh_nodes_sts[5] == self.NOT_OCCUPIED) :
          next_neighbours.append(neigh_nodes[5])
          self.neighbour_jump_offsets.append(neighbours_jump_offsets[2])
          next_costs.append(self.calc_node_cost(neigh_nodes[5])+ self.calc_dist_btw_nodes(cur_state,neigh_nodes[5],self.EUCLIDEAN_DIST))
      if len(next_neighbours) > 1:
        print("Neighbors =", next_neighbours)
      return next_neighbours, next_costs

    """###Initialise the next JPS search"""    
    def init_nxt_search(self):
      self.num_iteration = 0
      self.num_of_paths_checked += 1
      break_sts = False
      cur_path =[]
      cur_grid_node=[]
      cur_path_cost=[]

      #If no more paths to check
      if self.debug_en :
        print(self.num_of_paths_to_check,self.num_of_paths_checked,self.run_num, self.init_cond_flag)
      
      if self.num_of_paths_to_check == self.num_of_paths_checked:
        self.all_paths_analysed = True
        if self.init_cond_flag == self.PROJ_GOAL_REACHED:
          break_sts = True
      else:
        #Max paths to check
        if self.num_of_paths_to_check > self.MAX_PATHS_TO_CHECK:
          self.num_of_paths_checked = self.num_of_paths_to_check
          break_sts = True
          self.all_paths_analysed = True

        #Initialise
        self.old_move = 12
        self.num_iteration = 0
        
        #Get paths and costs
        nxt_paths = list(self.paths.keys())
        nxt_costs = list(self.costs.keys())

        #Delete traversed path 
        if self.run_num > 1:                 
          del self.paths[nxt_paths[0]]
          del self.costs[nxt_costs[0]]
          nxt_paths = list(self.paths.keys())
          nxt_costs = list(self.costs.keys())
          
        #Initialise for next search 
        if self.debug_en :  
          print(nxt_paths)
          print("Path num =", nxt_paths[0])  
        if len(nxt_paths) > 0:                    
          cur_path = list(self.paths[nxt_paths[0]])
          if cur_path != None:
            cur_grid_node = cur_path[-1]
            cur_path_cost = self.costs[nxt_costs[0]]                 
          self.run_num += 1
          self.visited_nodes_list = []
          
      #Clear goal reached flag
      self.reached_cur_goal = False
      if self.debug_en:
        print("Next cycle inputs:", cur_path)
      return cur_path, cur_grid_node, cur_path_cost,break_sts 

    """###Calculate node coordinates by offseting"""
    def get_node(self,cur_node,offset_idx):
      offset = self.coord_offset_dict[offset_idx]
      return [cur_node[0] + self.RESOLUTION*offset[0], cur_node[1] + self.RESOLUTION*offset[1], cur_node[2] + self.RESOLUTION*offset[2]]

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
      elif self.is_in_unknown(node):
        return self.COST_UNKNOWN
      else:
        return self.COST_EMPTY_CELL
    
    """###Calculate distance between nodes"""
    def calc_dist_btw_nodes(self,node1, node2, typ):
      if typ == self.MANHATTAN_DIST:
        dist = abs(node2[0] -node1[0]) + abs(node2[1] - node1[1]) + abs(node2[2] - node1[2])
      else:
        dist = math.sqrt((abs(node2[0] -node1[0])**2) + (abs(node2[1] - node1[1])**2) + (abs(node2[2] - node1[2])**2))
      return dist

    """###Calculate offset of a point in the bounding box grid"""
    def get_node_offset(self,node):
      P = int(self.BB_WIDTH/self.RESOLUTION) + 1
      i = int((node[0] - self.cur_state_in_grid[0] ) / self.RESOLUTION) 
      j = int((node[1] - self.cur_state_in_grid[1]) / self.RESOLUTION) 
      k = int((node[2] - self.cur_state_in_grid[2] ) / self.RESOLUTION) 
      num = int(self.ORIGIN_CELL_NUM + P*P*k + P*j + i)
      if self.debug_en:
        print("Offset for Node = " , node, num,i,j,k)
      return num
      
    """###Convert offsets back to coordinates"""
    def get_coordinates(self,offset,cur_position):
      off_position = cur_position    
      coordinates_off = (self.coord_offset_dict[offset])
      cur_offset = [off*self.RESOLUTION for off in coordinates_off]
      off_position = [cur_position[i]+cur_offset[i] for i in range(0,3)]    
      return off_position

    """###Check path is in free known space"""
    def is_path_valid(self, path):
      for node in path:      
        if self.is_occupied(node) == self.OCCUPIED:
          print("Occupied found at ", node)
          return 2, 0
        elif self.is_in_unknown(node):
          print("Unknown found at ", node)
          return 1, path.index(node)
      return 0, len(path)

    """###Check Occupancy status"""
    def is_occupied(self,node):
      num = self.get_node_offset(node)
      if self.occupancy_sts.values[num] == self.OCCUPIED:
        return self.OCCUPIED
      else:
        return self.NOT_OCCUPIED 

    """###Check whether a node is reachable or not"""
    def is_reachable(self,node):
      reachable_sts = False
      if self.is_occupied(node) == self.NOT_OCCUPIED:
        if (self.Z_LIMIT[0] < node[2] < self.Z_LIMIT[1]) and (self.Y_LIMIT[0] < node[1] < self.Y_LIMIT[1]) and (self.X_LIMIT[0] < node[0] < self.X_LIMIT[1]):
          reachable_sts = True
      return reachable_sts
        
    """###Check whether a node is already visited"""
    def has_visited(self,new_node):
      if self.visited_nodes_list.count(new_node) > 0:
        return True
      return False

    """###Check whether goal is inside Bounding Box"""
    def goal_within_bbx(self,goal_node, cur_node):
      if abs(goal_node[0]- cur_node[0]) <= self.BB_WIDTH/2 and abs(goal_node[1]- cur_node[1]) <= self.BB_WIDTH/2 and abs(goal_node[2]- cur_node[2]) < self.BB_WIDTH_Z/2 :
        return True
      return False

    """###Check whether goal is reachable"""
    def has_reached_goal(self, node1, node2):
      if self.calc_dist_btw_nodes(node1, node2, self.EUCLIDEAN_DIST) < self.ALLOWED_GOAL_POS_ERR :
        return True
      return False

    """###Check whether goal is behind the drone"""
    def goal_is_behind(self,cur_node, goal_node):
      if goal_node[0] < cur_node[0]:
        return True
      return False
      
    """####Check whether the nodes are same"""
    def is_same_node(self,node1, node2):
      if node1[0]-node2[0] < self.ALLOWED_POS_ERR and node1[1]-node2[1] < self.ALLOWED_POS_ERR and node1[2]-node2[2] < self.ALLOWED_POS_ERR:
        return True
      return False

    """###Check if node is in unknown space"""
    def is_in_unknown(self,node):
      num = self.get_node_offset(node)
      if self.occupancy_sts.values[num] == self.UNKNOWN:
        return True
      else:
        return False 
  
    """#Publish the Path"""

    def publish_global_plan(self):
      self.seq_cntr += 1            
      self.header.seq = self.seq_cntr
      self.header.stamp = rospy.Time.now()
      self.header.frame_id = 'world'
      self.global_nodes_path = Path()	    
      pose_list = []
      i = 0
      if self.debug_en == False:
        print("Global plan published : ", self.path_nodes_list)
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

    def test(self):
      cur_state = [3.0, 4.0, 3.5]
      cur_state = [3.0, 3.0, 3.5]
      #n = self.map_points.points.index([3.0, 4.0, 3.5])
      p = self.map_points.points[6386]
      print("Cell sts =",  self.is_occupied([p.x, p.y, p.z]), self.occupancy_sts.values[6386])
        
      #best_mv = self.select_moves(cur_state)
      print(self.get_neighbours(7,cur_state))

"""#Main module"""

if __name__ == '__main__':
    rospy.init_node('globalPlanner')
    pub_rate = rospy.Rate(20) #20 Hz
    try:
        globalPlanner_o = globalPlanner()
        #Subscribers        
        rospy.Subscriber("goal_loc", PointStamped, globalPlanner_o.read_goal)
        rospy.Subscriber("/SQ01s/state" , State, globalPlanner_o.read_state)
        rospy.Subscriber("grid_publisher" , PointCloud, globalPlanner_o.read_map)
        #Publishers
        while not rospy.is_shutdown():
          globalPlanner_o.get_next_state()
          pub_rate.sleep()

    except rospy.ROSInterruptException:  pass
