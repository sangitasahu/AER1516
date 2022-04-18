#!/usr/bin/env python

import math
import rospy
from snapstack_msgs.msg import Goal, State
from geometry_msgs.msg import PoseStamped, PointStamped, Point
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
        self.BB_WIDTH = 12  #Change for different map sizes
        self.SCALE_FACTOR_Z = 3
        self.BB_WIDTH_Z = self.BB_WIDTH/self.SCALE_FACTOR_Z
        self.STOP_DISTANCE = 1
        self.RESOLUTION = 0.25  #Can't go below this as the quadrotor is about 0.3m
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
        self.VALID_PATH = 0
        self.FIND_ALL_FEASIBLE_MOVES = 1
        self.FIND_NEXT_MOVE = 0
        #JPS related
        self.MAX_NUM_ITERATIONS = 250
        self.MAX_PATHS_TO_CHECK = 200
        self.MOVE_IDLE = 12
        #Moves      
        #Moves one cell ahead        
        self.MOVES_1 = [13,17,7,18,8,16,6,11]
        #Move up or down
        self.MOVES_z = [25,26,27,28]
        #Moves two cells ahead for robust collision avoidance
        self.MOVES_2 = [14,22,2,24,4,20,0,10]
        #Select moves for obstacle detection
        self.MOVES = self.MOVES_2
        

        """#Variables"""

        #Messages related
        self.header = Header()
        self.nxt_goal = Goal()
        self.global_nodes_path = Path()
        self.map_points = PointCloud()
        self.grid_occupancy_sts = ChannelFloat32()
        self.grid_occupancy_sts.name = 'grid occupancy'
        self.pub_map_points = PointCloud()
        self.occup_grid = PointCloud()

        #Map related
        self.x_range =[0,0]
        self.y_range =[0,0]
        self.z_range =[0,0]
        
        #JPS related 
        self.nxt_move_num = 0 #Move number
        self.num_of_cells = 0
        self.cur_goal = [10,10,3]      
        self.proj_cur_goal = [0,0,0]
        self.cur_state = [0,0,0]
        self.path_nodes_list = [[2,2,3]]
        self.cur_state_in_grid = [0,0,0]
        self.visited_nodes_list =[0,0,0]  
        self.paths = {}
        self.costs = {}
        self.num_of_paths_to_check = 1
        self.num_of_paths_checked = 0
        self.all_paths_analysed = False
        self.run_num = 0
        self.reached_cur_goal = False
        self.old_move = self.MOVE_IDLE
        self.occupancy_sts = []


        #Control flags and counters
        self.seq_cntr = 0
        self.num_iteration = 0
        self.init_cond_flag = self.PROJ_GOAL_REACHED
        self.jps_run_sts = self.COMPLETED
        self.goal_reached = True
        self.map_invalid = False
        
        #Timing and debugging
        self.start = 0
        self.end = 0
        self.debug_en = False
      
        #Publishers related
        self.global_path_pub = rospy.Publisher('global_plan', Path, queue_size=100)
        self.goal_pub = rospy.Publisher('/SQ01s/goal', Goal, queue_size=100)
        self.proj_goal_pub = rospy.Publisher('/projected_goal', PointStamped, queue_size=10)	
        self.global_grid_pub = rospy.Publisher('/global_grid',PointCloud, queue_size=2)			
       
        #Dictionaries
        self.coord_offset_dict = {
          0 : [ -2 , -2 , 0 ],
          1 : [ -1 , -2 , 0 ],
          2 : [ 0 , -2 , 0 ],
          3 : [ 1 , -2 , 0 ],
          4 : [ 2 , -2 , 0 ],
          5 : [ -2 , -1 , 0 ],
          6 : [ -1 , -1 , 0 ],
          7 : [ 0 , -1 , 0 ],
          8 : [ 1 , -1 , 0 ],
          9 : [ 2 , -1 , 0 ],
          10 : [ -2 , 0 , 0 ],
          11 : [ -1 , 0 , 0 ],
          12 : [ 0 , 0 , 0 ],
          13 : [ 1 , 0 , 0 ],
          14 : [ 2 , 0 , 0 ],
          15 : [ -2 , 1 , 0 ],
          16 : [ -1 , 1 , 0 ],
          17 : [ 0 , 1 , 0 ],
          18 : [ 1 , 1 , 0 ],
          19 : [ 2 , 1 , 0 ],
          20 : [ -2 , 2 , 0 ],
          21 : [ -1 , 2 , 0 ],
          22 : [ 0 , 2 , 0 ],
          23 : [ 1 , 2 , 0 ],
          24 : [ 2 , 2 , 0 ],
          25 : [0 , 0 , 1 ],
          26 : [0 , 0 , 2 ],
          27 : [0 , 0 , -1 ],
          28 : [0 , 0 , -2 ],
        }

        self.neighbor_offsets_dict = {
          13 : ([18,19,8,9],[23,3]),
          17 : ([16,21,18,23],[15,19]),
          7 : ([6,1,8,3],[5,9]),
          18 : ([17,22,13,14],[16,8]),
          8 : ([7,2,13,14],[6,18]),
          16 : ([17,22,11,10],[18,6]),
          6 : ([7,2,11,10],[8,16]),
          11 : ([16,15,6,5],[21,1]),          
          12 : [12,12] }
          

    """#Read the goal location for the drone"""
    def read_goal(self, msg):
      if ([msg.point.x, msg.point.y, msg.point.z] != self.cur_goal) and self.jps_run_sts == self.COMPLETED and self.goal_reached:
        #self.cur_goal = [msg.point.x, msg.point.y, msg.point.z]         
        self.goal_reached = False
        print("-----------------New goal received : ", self.cur_goal , "------------------")

    """#Read the current state of the drone"""      
    def read_state(self,msg):
      if self.jps_run_sts == self.COMPLETED:
        self.cur_state = [msg.pos.x,msg.pos.y, msg.pos.z]

    """####Read occup_grid1"""
    def read_obstacles(self,msg):
      if self.jps_run_sts == self.COMPLETED:
        #print("Read obstacles!") 
        self.occup_grid = msg

    """Inflate the obstacles"""
    def inflate_the_obstacles(self): 
      #print("Inflating obstacles!")
      self.new_occup_grid = PointCloud()
      for point in self.occup_grid.points :    
        if self.RESOLUTION >= 0.5:
          obs_offsets = self.MOVES_1
        else:
          obs_offsets = self.MOVES_1 + self.MOVES_2
        node = [point.x, point.y, point.z]    
        for offset in obs_offsets:
          next_node = self.get_coordinates(offset, node)              
          if self.is_occupied(next_node) != self.OCCUPIED:
            num = self.get_node_offset(next_node)                    
            self.occupancy_sts[num] = self.OCCUPIED
            cell = Point()
            cell.x, cell.y, cell.z = next_node[0], next_node[1],  next_node[2]
            self.new_occup_grid.points.append(cell)   
      self.publish_global_grid()

    """Generate the 3D Map from the point clouds containing occupancy info"""
    def read_map(self,pointClouds):
      if self.jps_run_sts == self.COMPLETED:
        #print("Reading Map!") 
        self.map_points.points = pointClouds.points
        self.grid_occupancy_sts.values = pointClouds.channels[0].values
        self.num_of_cells = len(self.grid_occupancy_sts.values)
        self.occupancy_sts = list(self.grid_occupancy_sts.values)
        #print("New map!")                     
        if len(pointClouds.points) == self.NUM_OF_CELLS: 
          #print("Empty : ", self.occupancy_sts.count(0), "Unknown: " ,self.occupancy_sts.count(-1), "Occupied : ", self.occupancy_sts.count(1), "Total : ", len(self.occupancy_sts))
          self.map_invalid = False 
          start_state = pointClouds.points[self.ORIGIN_CELL_NUM]
          self.x_range = [start_state.x , start_state.x + self.BB_WIDTH/2 ]
          self.y_range = [start_state.y - self.BB_WIDTH/2  , start_state.y + self.BB_WIDTH/2  ]
          self.z_range = [start_state.z - self.BB_WIDTH_Z/2 , start_state.z + self.BB_WIDTH_Z/2 ]
          self.cur_state_in_grid = [start_state.x, start_state.y, start_state.z]             
        else:   
          self.map_invalid = True      
          print("Length of grid received is ", len(pointClouds.points), "which is invalid !")
          print("Looking for ", self.NUM_OF_CELLS , " points!")
          print("Correctly set the BB_WIDTH and RESOLUTION variables!")

    """###Calculate the next state"""
    def get_next_state(self):
      if not self.map_invalid and self.jps_run_sts == self.COMPLETED and len(self.occupancy_sts) == self.NUM_OF_CELLS:
        # not self.goal_reached and 
        start_time_jps = time.time()
        self.jps_run_sts = self.RUNNING
        #Store a copy of the variables
        if self.debug_en :          
          print("JPS started searching...", self.cur_state_in_grid)
        #Initialise
        self.path_nodes_list = [self.cur_state]
        #Get the projected goals in unknown space and known free space
        print("Actual state :", self.cur_state, "Current state from mapper :", self.cur_state_in_grid, "Goal Location :" ,self.cur_goal)
        self.inflate_the_obstacles()
        #Goal is behind the drone
        if self.goal_is_behind(self.cur_state_in_grid,self.cur_goal):
          self.get_projected_goal()
          self.path_nodes_list.append(self.unk_cur_goal)
        else: #Calculate JPS path
          self.get_projected_goal()
          if self.goal_is_reachable:          
            next_states = self.find_jps_path()  
            if len(next_states) > 2:
              if self.debug_en == False :
                print("Global path found by JPS! " )        
              val_sts = self.is_path_valid(next_states)
              if val_sts == self.VALID_PATH:
                for node in next_states:
                  self.path_nodes_list.append(node)
            else:
              if self.debug_en == False :
                print("No path found by JPS! " )  
        #Publish the global plan
        self.publish_global_plan()
        #If end goal is reached
        if self.has_reached_goal(self.path_nodes_list[-1],self.cur_goal):
            self.goal_reached = True
            print("---------------------Reached end goal!----------------------")
        elif not self.goal_is_reachable:
            self.goal_reached = True
            print("-------------------End goal is unreachable!---------------------")
        elif self.goal_is_reachable and self.path_nodes_list[-1] == self.cur_state_in_grid :
          print("-------------------JPS can't find any path!-------------------")
          self.goal_reached = True
        end_time_jps = time.time()
        if self.debug_en :
          print("Time taken by JPS is ", end_time_jps - start_time_jps)
        self.jps_run_sts = self.COMPLETED
        self.visited_nodes_list = []							

    """###Get projected goal for current JPS iteration"""
    def get_projected_goal(self):
      self.start_p = time.time()
      min_d_goal = self.INFINITY
      goal_within_bbx = self.goal_within_bbx(self.cur_goal, self.cur_state_in_grid) 
      goal_in_unknown = self.is_occupied(self.cur_goal) == self.UNKNOWN
      if goal_within_bbx and not goal_in_unknown:              
        if self.is_occupied(self.cur_goal)==self.NOT_OCCUPIED:
          print("Goal within the bounding box and is reachable!")          
          self.goal_is_reachable = True
          self.proj_cur_goal = self.cur_goal
          self.unk_cur_goal = self.cur_goal 
        else:
          print("Goal within the bounding box but is not reachable!")
          self.goal_is_reachable = False
          self.proj_cur_goal = self.cur_state_in_grid
          self.unk_cur_goal = self.cur_state_in_grid
      else:
        behind_goal = self.goal_is_behind(self.cur_state_in_grid,self.cur_goal)
        if (goal_within_bbx and goal_in_unknown) :
          self.goal_is_reachable = True
          occ_sts = self.NOT_OCCUPIED
          print("Goal within the bounding box but is in the unknown space!")
        elif behind_goal :
          self.goal_is_reachable = False
          occ_sts = self.UNKNOWN
          print("Goal in the unknown space behind the drone!")
        else:
          self.goal_is_reachable = True
          occ_sts = self.NOT_OCCUPIED


        not_occupied_cells = []
        self.PROJ_GOAL_RADIUS = 5
        #Get the nearest unknown node to the goal
        for i in range(0, self.num_of_cells):
          cur_point = [self.map_points.points[i].x, self.map_points.points[i].y, self.map_points.points[i].z]
          if self.occupancy_sts[i] == occ_sts and (cur_point[2]==self.cur_goal[2]):
            d_goal = self.calc_dist_btw_nodes(cur_point, self.cur_goal,self.EUCLIDEAN_DIST)
            if d_goal <= min_d_goal and d_goal < self.PROJ_GOAL_RADIUS:
              min_d_goal = d_goal
              self.proj_cur_goal = cur_point
          elif self.occupancy_sts[i] == self.NOT_OCCUPIED and not behind_goal:
            not_occupied_cells.append(cur_point)

        self.unk_cur_goal = self.proj_cur_goal 

      self.end_p = time.time()
      if self.debug_en :
        print("Time to read through the map : ", self.end_p - self.start_p)
        print("Unknown and Known projected goals are " , self.unk_cur_goal ,self.proj_cur_goal)

    """###Find paths"""
    def find_jps_path(self):
      best_path = [self.cur_state_in_grid]
      
      #Set the height first
      if self.cur_goal[2] - self.cur_state_in_grid[2] != 0: 
        best_path.append([self.cur_state_in_grid[0],self.cur_state_in_grid[1],self.cur_goal[2]] )
        self.cur_state_in_grid = best_path[-1]

      #Iterate over different moves and save the path with minimum cost
      min_cost = self.INFINITY
      self.cur_best_path_cost = self.INFINITY
      best_move, feasible_moves = self.select_moves(self.cur_state_in_grid,self.FIND_ALL_FEASIBLE_MOVES)
      if self.debug_en:
        print("Feasible moves =", feasible_moves)
      for move in feasible_moves:
        self.cur_move_num = move
        if self.debug_en:
          print("Current move =", move)        
        path, cost = self.get_jps_successors()
        if len(path) != 0 and cost < min_cost:
          best_path = path
          min_cost = cost

      return best_path


    """###Calculate jps jump nodes"""
    def get_jps_successors(self):
      #Initialise JPS
      self.visited_nodes_list = []
      self.num_iteration = 0      
      self.paths = {}
      self.costs = {}
      self.num_of_paths_to_check = 1
      self.num_of_paths_checked = 0
      self.all_paths_analysed = False
      self.run_num = self.num_of_paths_to_check
      self.reached_cur_goal = False
      self.old_move = self.MOVE_IDLE
      cur_grid_node = self.cur_state_in_grid
      possible_paths_dict ={}     
      cur_path_cost = 0
      cur_path = [] 
      valid_path_costs =[]  
      self.nxt_move_num = self.cur_move_num  
      
      #Find all possible paths
      while self.num_of_paths_to_check - self.num_of_paths_checked > 0 :
        if self.debug_en : 
          print("----------------Run no:", self.run_num) 
        nxt_state =  self.get_coordinates(self.nxt_move_num, cur_grid_node)           
        if self.debug_en :
          print("Next state = ", nxt_state,self.is_reachable(nxt_state), self.is_occupied(nxt_state))
          print("Move_num = ", self.nxt_move_num)
        if self.is_reachable(nxt_state) and self.nxt_move_num != self.MOVE_IDLE:          
          next_nodes , next_costs = self.get_neighbours(self.nxt_move_num, cur_grid_node)
          if self.debug_en:
            print("Next nodes = ", next_nodes,self.reached_cur_goal)
          
          if self.reached_cur_goal == False:
            if len(next_nodes)>0:
              if self.debug_en:
                print("Number of forced neighbours =", len(next_nodes)-1,nxt_state, self.nxt_move_num, next_nodes)
              for i in range(0,len(next_nodes)):
                if self.is_reachable(next_nodes[i]):
                  if i==0:
                    cur_grid_node = next_nodes[i]
                    cur_path_cost += next_costs[i]
                    if cur_path_cost > self.cur_best_path_cost:
                      self.num_iteration = self.MAX_NUM_ITERATIONS
                    else:
                      if self.old_move == self.nxt_move_num and self.old_move != self.MOVE_IDLE and len(next_nodes)==1 and len(cur_path) > 1:                  
                        cur_path[-1] = cur_grid_node
                      else:
                        cur_path.append(cur_grid_node)                               
                  else:
                    cost_of_cur_grid = self.calc_node_cost(next_nodes[i]) 
                    self.num_of_paths_to_check += 1                     
                    updated_cur_path = list(cur_path) 
                    if len(cur_path) > 0:
                      del updated_cur_path[-1]
                    updated_cur_path.append(next_nodes[i])
                    self.paths[self.num_of_paths_to_check] =   updated_cur_path  
                    self.costs[self.num_of_paths_to_check] = cur_path_cost+cost_of_cur_grid
              if self.debug_en:
                print(cur_path, updated_cur_path)
                print("Paths N Costs" , self.paths, self.costs)
                print("Next cur state in grid = ", cur_grid_node )
              self.visited_nodes_list.append(cur_grid_node)  #Can grow large
              self.num_iteration += 1
              if self.num_iteration == self.MAX_NUM_ITERATIONS - 1:
                if self.debug_en:
                  print("Max iterations reached!")
                  break
                self.init_cond_flag = self.MAX_ITERATIONS_REACHED
                cur_path, cur_grid_node, cur_path_cost,break_sts  = self.init_nxt_search()
                if break_sts:
                  break
          else:
            self.init_cond_flag = self.PROJ_GOAL_REACHED

            if self.old_move == self.nxt_move_num and self.old_move != self.MOVE_IDLE and len(cur_path) >0:                  
              cur_path[-1] = self.proj_cur_goal
            else:
              cur_path.append(self.proj_cur_goal)
            self.visited_nodes_list.append(self.proj_cur_goal)
            valid_path_costs.append(cur_path_cost) 
            val_sts = self.is_path_valid(cur_path)
            if val_sts == self.VALID_PATH and len(cur_path) > 0:              
              possible_paths_dict[cur_path_cost] = list(cur_path) 
            if cur_path_cost < self.cur_best_path_cost :
              self.cur_best_path_cost  = cur_path_cost            
            if self.debug_en:
              print("Projected goal reached!")
              print(self.cur_state,self.cur_state_in_grid,self.proj_cur_goal,cur_path,cur_path_cost)
              print("Number of paths checked :",self.num_of_paths_checked + 1)
            cur_path, cur_grid_node, cur_path_cost,break_sts = self.init_nxt_search()
            if break_sts:
              break
        else:                    
          cur_path, cur_grid_node, cur_path_cost, break_sts  = self.init_nxt_search()
          if self.debug_en:
            print("No path possible!",break_sts)
          if break_sts:
              break
        self.old_move = self.nxt_move_num
        self.nxt_move_num , moves = self.select_moves(cur_grid_node,self.FIND_NEXT_MOVE)  
		    
        #Debug publish code
        if self.debug_en:         
          if self.init_cond_flag == self.PROJ_GOAL_REACHED:
            self.path_nodes_list =[]
            for node in cur_path:
              self.path_nodes_list.append(node)
            self.publish_global_plan()
          
      if self.debug_en:
        print("Possible paths =",possible_paths_dict)

      #Chose the best path
      best_path = []
      min_cost = self.INFINITY
      if len(valid_path_costs) > 0:
        path_costs = valid_path_costs
      else:
        if self.debug_en :
          print("No path found in free known space!")
        path_costs = list(possible_paths_dict.keys())
      if self.all_paths_analysed and len(path_costs)>0:        
        min_cost = min(path_costs)
        best_path = possible_paths_dict[min_cost]
      
      if self.debug_en :
        print("All paths analysed =",self.all_paths_analysed)
        print("Best Path = ", best_path, "Best cost =", min_cost)

      return best_path, min_cost
    
    """###Select the Best Move"""
    def select_moves(self, cur_state,mode):
      min_dist = self.INFINITY
      best_move = self.MOVE_IDLE
      mov_idx = 0
      feasible_moves = []
      old_dz = self.INFINITY

      #Check all moves    
      for mov_idx in range(0, len(self.MOVES)):
        next_node = self.get_coordinates(self.MOVES_1[mov_idx], cur_state)
        next_next_node = self.get_coordinates(self.MOVES_2[mov_idx], cur_state)
        old_dz = next_node[2] - cur_state[2]
        if self.debug_en:
          print(self.MOVES_1[mov_idx],cur_state,next_node,self.is_reachable(next_node),self.is_occupied(next_node),self.has_visited(next_node),next_next_node,self.is_reachable(next_next_node),self.has_visited(next_next_node),self.is_occupied(next_next_node),self.calc_dist_btw_nodes(next_node, self.proj_cur_goal, self.MANHATTAN_DIST))
        if self.is_reachable(next_node) == True and self.has_visited(next_node) == False and self.is_reachable(next_next_node) == True and self.has_visited(next_next_node) == False and old_dz < 2*self.RESOLUTION:
          dist = self.calc_dist_btw_nodes(next_node, self.proj_cur_goal, self.MANHATTAN_DIST)
          if dist < min_dist:
            min_dist = dist
            best_move = self.MOVES_1[mov_idx]
            feasible_moves.insert(0,self.MOVES_1[mov_idx])
          else:
            feasible_moves.append(self.MOVES_1[mov_idx])
      if self.debug_en : 
        print("Best move =", best_move )
      return best_move, feasible_moves
    

    """####Check move feasibility"""
    def is_good_move(self,old_move, best_move, cur_state):
      next_state = self.get_coordinates(old_move,cur_state)
      if next_state[0] < cur_state[0]:
        return False
      else:
        return True
    
    """###Get neighbours for a move"""
    def get_neighbours(self,nxt_move_num,cur_state):
      next_neighbours = []      
      neigh_nodes = []
      next_costs = []
      neigh_nodes_sts =[]
     
      #Evaluate neighbours
      neighbours_to_chk = self.neighbor_offsets_dict[nxt_move_num][0]
      next_neighbours_to_chk = self.neighbor_offsets_dict[nxt_move_num][1]
      if self.debug_en:
        print("Neigh to chk",neighbours_to_chk )
      num_of_neighbors = len(neighbours_to_chk)
      for neigh_node_idx in neighbours_to_chk:
        neigh_node_coord = self.get_coordinates(neigh_node_idx, cur_state)
        if self.debug_en:
          print(nxt_move_num,neigh_node_idx,cur_state,neigh_node_coord,self.has_reached_goal(neigh_node_coord,self.proj_cur_goal))
        neigh_nodes.append(neigh_node_coord)
        neigh_nodes_sts.append(self.is_reachable_neighbour(neighbours_to_chk.index(neigh_node_idx),cur_state,neigh_node_coord,next_neighbours_to_chk))
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
        if neigh_nodes_sts[1] == self.OCCUPIED and neigh_nodes_sts[0] == self.NOT_OCCUPIED :
          next_neighbours.append(neigh_nodes[0])
          next_costs.append(self.calc_node_cost(neigh_nodes[0]) + self.calc_dist_btw_nodes(cur_state,neigh_nodes[0],self.EUCLIDEAN_DIST))
      else:
        if neigh_nodes_sts[1] == self.OCCUPIED and neigh_nodes_sts[0] == self.NOT_OCCUPIED :
          next_neighbours.append(neigh_nodes[0])
          next_costs.append(self.calc_node_cost(neigh_nodes[0]) + self.calc_dist_btw_nodes(cur_state,neigh_nodes[0],self.EUCLIDEAN_DIST))
        if neigh_nodes_sts[3] == self.OCCUPIED and neigh_nodes_sts[2] == self.NOT_OCCUPIED :
          next_neighbours.append(neigh_nodes[2])
          next_costs.append(self.calc_node_cost(neigh_nodes[2])+ self.calc_dist_btw_nodes(cur_state,neigh_nodes[2],self.EUCLIDEAN_DIST))

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
      elif self.num_of_paths_checked > self.MAX_PATHS_TO_CHECK:
        #Max paths to check        
        self.num_of_paths_checked = self.num_of_paths_to_check
        break_sts = True
        self.all_paths_analysed = True
      else:
        #Initialise for next search
        self.old_move = self.MOVE_IDLE
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
            self.visited_nodes_list = []
            for node in cur_path:
              self.visited_nodes_list.append(node)                 
          self.run_num += 1          
          
      #Clear goal reached flag
      self.reached_cur_goal = False
      if self.debug_en:
        print("Next cycle inputs:", cur_path, self.all_paths_analysed,break_sts)
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
      elif self.is_occupied(node) ==  self.UNKNOWN:
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
      if num > self.NUM_OF_CELLS and self.debug_en:
        print( "Node ", node , "Offset =", num , " is out of the bounding box with", self.NUM_OF_CELLS, " cells." "i,j,k =",i,j,k)
        print("Current state =", self.cur_state_in_grid)
      return num
      
    """###Convert offsets back to coordinates"""
    def get_coordinates(self,offset,cur_position):
      off_position = cur_position    
      coordinates_off = (self.coord_offset_dict[offset])
      cur_offset = [off*self.RESOLUTION for off in coordinates_off]
      off_position = [cur_position[i]+cur_offset[i] for i in range(0,3)]    
      return off_position

    """###Check if reachable next neighbour"""
    def is_reachable_neighbour(self,move_idx,cur_node,neigh_node,next_neighbours):
      if move_idx == 1 or move_idx == 3 :
        return self.is_reachable(neigh_node)
      else:
        if self.is_reachable(neigh_node):
          next_node_idx = next_neighbours[int((move_idx)/2)]
          offset = self.coord_offset_dict[next_node_idx]
          next_next_node = [cur_node[0]+self.RESOLUTION*offset[0],cur_node[1]+self.RESOLUTION*offset[1],cur_node[2]+self.RESOLUTION*offset[2]]				
          if self.is_reachable(next_next_node):
            return True 
        return False

    """###Check path is in free known space"""
    def is_path_valid(self, path):
      for node in path:      
        if self.is_occupied(node) == self.OCCUPIED:
          print("Occupied found at ", node)
          return self.OCCUPIED
        elif self.is_occupied(node) == self.UNKNOWN:
          print("Unknown found at ", node)
          return self.UNKNOWN
      return self.NOT_OCCUPIED

    """###Check Occupancy status"""
    def is_occupied(self,node):
      num = self.get_node_offset(node)
      if num < self.NUM_OF_CELLS:
        if self.occupancy_sts[num] == self.OCCUPIED:
          return self.OCCUPIED
        elif self.occupancy_sts[num] == self.UNKNOWN:
          return self.UNKNOWN
        else:
          return self.NOT_OCCUPIED 
      else:
        #print("Node is outside of the bounding box")
        return self.OCCUPIED
    
    """###Check whether a node is reachable or not"""
    def is_reachable(self,node):
      reachable_sts = False
      if self.is_occupied(node) == self.NOT_OCCUPIED:
        if (self.z_range[0] <= node[2] <= self.z_range[1]) and (self.y_range[0] <= node[1] <= self.y_range[1]) and (self.x_range[0] <= node[0] <= self.x_range[1]):
          reachable_sts = True
      #if node == [6.0, 4.0, 2.0] :
        #print("Not Reachable point : [6.0, 4.0, 2.0]",self.is_occupied(node))
      return reachable_sts
        
    """###Check whether a node is already visited"""
    def has_visited(self,new_node):
      if self.visited_nodes_list.count(new_node) > 0:
        return True
      return False
    
    """###Check whether next node is feasible"""
    def is_feasible_node(self,node,goal):
      if node == [5.0, 2.5, 3.0]:
        print(node,goal)
      if node[0]>goal[0] or (node[1]>0 and node[1]>goal[1])  or (node[1]<0 and node[1]<goal[1]) or (node[2]>0 and node[2]>goal[2])  or (node[2]<0 and node[2]<goal[2]) :
           return False
      if node == [5.0, 2.5, 3.0]:
        print(True)
      return True					

    """###Check whether goal is inside Bounding Box"""
    def goal_within_bbx(self,goal_node, cur_node):
      if abs(goal_node[0]- cur_node[0]) <= self.BB_WIDTH/2 and abs(goal_node[1]- cur_node[1]) <= self.BB_WIDTH/2 and abs(goal_node[2]- cur_node[2]) <= self.BB_WIDTH_Z/2 :
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
  
    """#Publish the Path"""

    def publish_global_plan(self):      
      self.seq_cntr += 1            
      self.header.seq = self.seq_cntr
      self.header.stamp = rospy.Time.now()
      self.header.frame_id = 'world'
      self.global_nodes_path = Path()    
      pose_list = []
      i = 0
      if self.debug_en :
        print("Global plan published : ", self.path_nodes_list)

      #Fill global plan buffer
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
        node_pose.header.frame_id = 'vicon'
        pose_list.append(node_pose)
        i += 1

      #Publish global plan
      self.global_nodes_path.header= self.header
      self.global_nodes_path.poses = pose_list
      self.global_path_pub.publish(self.global_nodes_path)
      
      #Publish projected and unknown goals
      proj_goal_point = PointStamped()
      proj_goal_point.point.x = self.proj_cur_goal[0]
      proj_goal_point.point.y = self.proj_cur_goal[1]
      proj_goal_point.point.z = self.proj_cur_goal[2]
      proj_goal_point.header = self.header
      self.proj_goal_pub.publish(proj_goal_point)

      #Total JPS time							 
      self.end = time.time()
      if self.debug_en:
        rospy.loginfo(self.end -  self.start)

    """#Publish the Goal"""
    def publish_sim_path(self):
      self.seq_cntr += 1 
                 
      self.header.seq = self.seq_cntr
      self.header.stamp = rospy.Time.now()
      self.header.frame_id = 'vicon'
      self.nxt_goal.header = self.header   

      if len(self.path_nodes_list) > 1:
        path_node = self.path_nodes_list[1]
      else:
        path_node = self.path_nodes_list[0]
      
      self.nxt_goal.p.x = path_node[0]
      self.nxt_goal.p.y = path_node[1]
      self.nxt_goal.p.z = path_node[2]
      self.nxt_goal.v.x = 2.5

      dy = path_node[1]-self.cur_state[1]
      dx = path_node[0]-self.cur_state[0]	
      self.nxt_goal.yaw = math.atan2(dy,dx)       

      self.goal_pub.publish(self.nxt_goal)


    """###Publish global grid after inflation"""
    def publish_global_grid(self):
      self.header.seq = self.seq_cntr
      self.header.stamp = rospy.Time.now()
      self.header.frame_id = 'vicon'
      self.pub_map_points.header = self.header 
      self.pub_map_points.points = self.new_occup_grid.points
      self.global_grid_pub.publish(self.pub_map_points)
      

"""#Main module"""

if __name__ == '__main__':
    rospy.init_node('globalPlanner')
    pub_rate = rospy.Rate(1) #20 Hz
    try:
        globalPlanner_o = globalPlanner()
        #Subscribers        
        rospy.Subscriber("goal_loc", PointStamped, globalPlanner_o.read_goal)
        rospy.Subscriber("/SQ01s/state" , State, globalPlanner_o.read_state)
        rospy.Subscriber("grid_publisher" , PointCloud, globalPlanner_o.read_map)
        rospy.Subscriber("/occup_grid1", PointCloud, globalPlanner_o.read_obstacles)
        
        #Publishers
        globalPlanner_o.path_nodes_list = [[2,2,3],[2,2,3]]
        while not rospy.is_shutdown():
          globalPlanner_o.get_next_state()
          globalPlanner_o.publish_sim_path()
          pub_rate.sleep()

    except rospy.ROSInterruptException:  pass