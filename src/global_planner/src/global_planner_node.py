#!/usr/bin/env python

import math
import rospy
from snapstack_msgs.msg import Goal, State
from geometry_msgs.msg import PoseStamped, Vector3
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud, ChannelFloat32
from std_msgs.msg import Header, Int32MultiArray
import random

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
	#self.timer = rospy.Timer(rospy.Duration(0.5), self.pub_spin)
        #Constants
        self.DIAGONAL = 1
        self.STRAIGHT = 2
        self.BB_WIDTH = 5
        self.EUCLIDEAN_DIST = 1
        self.MANHATTAN_DIST = 0
        self.INFINITE_COST = 1000
        self.OCCUPIED = 1
        self.NOT_OCCUPIED = 0
        self.GRID_WIN_WIDTH = 3 #Keep it even
        self.D_LIMIT = math.floor((self.BB_WIDTH - 1)/2)
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
        self.move = self.STRAIGHT #Can be straight or diagonal
        self.nxt_move_num = 0 #Move number
        self.num_of_cells = 0 
        self.cur_goal = Vector3()
        self.cur_state = Vector3()
        self.cur_node = []
        self.new_nodes = []
        self.map_points = PointCloud()
        self.occupancy_sts = ChannelFloat32()
        self.occupancy_sts.name = 'occupancy probability'
        self.global_path = Path()
        self.path_nodes_list = []
        self.map3D =[]
        self.seq_cntr = 0
        self.stop_jps = 0

        #Publishers
        self.global_path_pub = rospy.Publisher('global_plan', Path, queue_size=100)
    	
	#Subscribers        
        #rospy.Subscriber("/SQ01s/goal" , Goal, self.read_goal)
        #rospy.Subscriber("/SQ01s/state" , State, self.read_state)
        #rospy.Subscriber("probability_publisher" , PointCloud, self.read_map)
        #rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.read_rviz_goal)
       
        #Offsets
        self.neigh_cell_num_dict = {
            #Straight
            0 : [13,7,1,16,10,22,25,19],  #forward
            1 : [7,13,16],                #left
            2 : [1,13,11],                #right
            3 : [5,1,7,2,8],              #up
            4 : [3,0,6,1,7],              #down    
            #Diagonals
            5:  [0,1,3],                  #right down
            6:  [2,1,5],                  #right up
            7:  [8,5,7],                  #left down
            8:  [6,3,7],                  #left up
            9:  [16,7,13],                #forward left
            10:  [10,1,13],               #forward right
            11:  [17,4,5,8,7,13,16,14],   #forward left up
            12:  [11,4,1,2,5,10,14,13],   #forward right up
            13:  [9,4,1,0,3,10,13,12],    #forward right down   
            14:  [15,4,7,3,6,13,12,16],   #forward left down
            15:  [14,4,5,8,7,1,2,10,11,13,16,17], #forward up
            16:  [12,1,4,7,0,3,6,10,13,16,9,15]  #forward down
          }

    """#Read the goal and State"""

    def read_rviz_goal(self,msg):
      rospy.loginfo("New goal!")    
      self.cur_rviz_goal = [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]
      rospy.loginfo(self.cur_rviz_goal)
      self.seq_cntr += 1
      if self.seq_cntr > 100:
        self.seq_cntr = 0

    def read_goal(self,msg):
      self.cur_goal = msg.p
      self.dist_to_goal = self.calc_dist_btw_nodes(self.cur_state, self.cur_goal)
      rospy.loginfo(self.cur_goal)

    def read_state(self,msg):
      self.cur_state = msg.pos
      self.cur_node = [self.cur_state.x,self.cur_state.y, self.cur_state.z, self.NOT_OCCUPIED, self.COST_EMPTY_CELL ]
      self.path_nodes_list.append(self.cur_node)
      rospy.loginfo(self.cur_state)

    """Generate the 3D Map from the point clouds containing occupancy and costs (5D array)

    ###Read the 3D grid
    """

    def read_map(self,pointClouds):
      self.map_points.points = pointClouds.points
      self.occupancy_sts.values = pointClouds.channels[0].values    

      #self.map3D = self.generate_map_3D()

    """###Convert from Point to list"""

    def p2n(point):
      return [point.x, point.y, point.z]

    """###Calculate cell cost"""

    def get_cell_cost(self, node, occ_sts):
      if occ_sts:
        return self.COST_OCCUPIED_CELL
      elif (node[0]==self.D_LIMIT[1]) or (node[1]==self.X_LIMIT[0] or node[1]==self.D_LIMIT[1]) or (node[2]==self.X_LIMIT[0] or node[2]==self.D_LIMIT[1]):
        return self.COST_WALL
      elif (node == self.p2n(self.cur_goal)):
        return self.COST_GOAL
      else:
        return self.COST_EMPTY_CELL

    """###Calculate distance between nodes"""

    def calc_dist_btw_nodes(self,node1, node2, typ):
      if typ == self.MANHATTAN_DIST:
        dist = math.abs(node2[0] -node1[0]) + math.abs(node2[1] - node1[1]) + math.abs(node2[2] - node1[2])
      else:
        dist = (math.abs(node2[0] -node1[0])**2) + (math.abs(node2[1] - node1[1])**2) + (math.abs(node2[2] - node1[2])**2)
      return dist

    """###Calculate cost between nodes"""

    def calc_cost_btw_nodes(self,node1, node2):
                return node1[4] + self.calc_dist_btw_nodes(node1, node2, self.MANHATTAN_DIST) + node2[4]

    """###Move the node by an offset"""

    def move_node_by_offset(self, node, offset):
      new_node = [node[0]+offset[0], node[1]+offset[0] , node[2]+offset[2], 0,0]
      occ_sts = self.get_occupancy_sts(new_node)
      cell_cost = self.get_cell_cost(new_node,occ_sts)
      return [new_node[0], new_node[0] , new_node[2], occ_sts, cell_cost]

    """###Get the occupancy status"""

    def get_occupancy_sts(self,node):
      node_idx = 0
      occ_sts = self.OCCUPIED #For nodes not in the map, the status is OCCUPIED
      for point in self.map_points.points:
        if point == node[0:2]:
          occ_sts = self.occupancy_sts.values[node_idx] 
      return occ_sts

    """###Generate the 3D Map"""

    def generate_map_3D(self):
      self.map = []
      for k in range(self.Z_LIMIT[0], self.Z_LIMIT[1]): 
        for j in range(self.Y_LIMIT[0], self.D_LIMIT[1]):
          for i in range(self.X_LIMIT[0], self.X_LIMIT[1]):
            node = [self.cur_node[0] + i, self.cur_node[1] + j, self.cur_node[1] + k ]
            occ_sts = self.get_occupancy_sts(node)
            cell_cost = self.get_cell_cost(node,occ_sts)
            self.map.append([node[0], node[1], node[2], occ_sts,cell_cost])
      return self.map

    """#Decide the next move(Access the 17 cells in the front)"""

    def select_move(self):
      offsets = [[1,0,0],   #forward
             [0,1,0],   #left
             [0,-1,0],  #right
             [0,0,1],   #up
             [0,0,-1],  #down
             [1,1,0],   #forward left
             [1,-1,0],  #forward right
             [1,1,1],   #forward left up
             [1,1,-1],  #forward left down
             [1,-1,1],  #forward right up
             [1,-1,-1], #forward right down
             [1,0,1],   #forward up
             [1,0,-1],  #forward down
             [0,1,1],   #left up
             [0,1,-1],  #left down
             [0,-1,1],  #right up
             [0,-1,-1]] #right down

      node_c = self.cur_node[0:2]; 
      move_num = 0
      for offset in offsets:
        move_num += 1
        node_s = node_c + offset
        occ_s = self.get_occupancy_sts(node_s)
        if occ_s == self.NOT_OCCUPIED:
          cost = self.get_cell_cost(node_s, occ_s) + self.calc_cost_btw_nodes(node_c,node_s,self.EUCLIDEAN_DIST)
          if cost < min_cost:
            min_cost = cost
            self.nxt_move_num = move_num
            next_node = node_s

      if self.move_type < 6:
        self.nxt_move = self.STRAIGHT
      else:
        self.nxt_move = self.DIAGONAL

      return next_node

    """#Get neighbours for the next node"""

    def get_neighbours(self,nxt_node, move_num):    
      neighbours = []
      obstacles_sts = False
      #Add the next probable state to the neighbours list and prune nodes around it        
      cell_nums = self.neigh_cell_num_dict.get(move_num)
      node = None
      if self.move == self.STRAIGHT:
            forced_node = None
            for cell_num in cell_nums:
              node = self.map[cell_num]
              if node[4] == self.OCCUPIED:
                dist = self.calc_cost_btw_nodes(self.cur_node, node)
                if dist == 3 and obstacles_sts==False:
                   neighbours.append(self.move_node_by_offset(node,[-1,0,0,0,0]))
                elif dist == 2:
                   forced_node = node
                   obstacles_sts = True
                   break
              else:
                 if obstacles_sts == True and forced_node != None:
                     dist_o = self.calc_cost_btw_nodes(node, forced_node)
                     if dist_o == 1:
                         neighbours.append(node)
      else:
        pass
      neighbours.append(nxt_node)  
      return neighbours

    """#Add final jump points"""

    def get_next_state(self):
      count = 0
      self.path_nodes_list = []
      while (count < self.BB_WIDTH):
        count += 1
        nxt_state = self.select_move()
        next_jump_nodes = self.get_neighbours(nxt_state)
        if len(next_jump_nodes) > 1 :
          self.path_nodes_list.append(self.cur_node)
          self.cur_state = nxt_state
        elif next_jump_nodes[0]== self.p2n(self.cur_goal):
          self.cur_state = self.cur_goal
          break
        else:
          self.cur_state = next_jump_nodes[0]

    def fill_dummy_path(self):
      self.path_nodes_list = []
      for i in range(0,4):
          self.path_nodes_list.append([random.uniform(0,2)*i, random.uniform(-1,1)*i, random.uniform(0,1)*i])
		#self.path_nodes_list.append([0.75*i, 1.2*i, 0.5*i])

    """#Publish the Path"""

    def publish_global_path(self):            
      self.header.seq = self.seq_cntr
      self.header.stamp = rospy.Time.now()
      self.header.frame_id = 'world'
      self.global_nodes_path = Path()	    
      self.fill_dummy_path()
      pose_list = []
      i = 0
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


"""#Main module"""

if __name__ == '__main__':
    rospy.init_node('globalPlanner')
    try:
        globalPlanner_o = globalPlanner()
        rate = rospy.Rate(1)
#Subscribers        
        rospy.Subscriber("/SQ01s/goal" , Goal, globalPlanner_o.read_goal)
        rospy.Subscriber("/SQ01s/state" , State, globalPlanner_o.read_state)
        rospy.Subscriber("probability_publisher" , PointCloud, globalPlanner_o.read_map)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, globalPlanner_o.read_rviz_goal)
        while (not rospy.is_shutdown()):
          globalPlanner_o.publish_global_path()
          rate.sleep()
	#Infinite Loop
    except rospy.ROSInterruptException:  pass