#!/usr/bin/env python

import math
import rospy
from map_simulator.msg import Map3D
from snapstack_msgs.msg import Goal, State
from geometry_msgs.msg import PoseStamped, Pose, Vector3
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud, ChannelFloat32

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
		self.BB_WIDTH = 5
		self.EUCLIDEAN_DIST = 1
		self.MANHATTAN_DIST = 0
		self.INFINITE_COST = 1000
		self.OCCUPIED = 1
		self.NOT_OCCUPIED = 0
		self.GRID_WIN_WIDTH = 3 #Keep it even
		#Variables
		self.move = self.STRAIGHT
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

		#Publishers
		self.global_path_pub = rospy.Publisher('global_path', Path, queue_size=10)
		#Subscribers		
		self.goal_sub = rospy.Subscriber("/SQ01s/goal" , Goal, self.read_goal)
		self.state_sub = rospy.Subscriber("/SQ01s/state" , State, self.read_state)
		self.mapper_sub = rospy.Subscriber("probability_publisher" , PointCloud, self.read_map)
		self.rviz_goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.read_rviz_goal)
	
	def read_rviz_goal(self, msg):	
		self.cur_rviz_goal = [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]
		rospy.loginfo(self.cur_rviz_goal)

	def read_goal(self,msg):
		self.cur_goal = msg.p;
		self.dist_to_goal = self.calc_dist_btw_nodes(self.cur_state, self.cur_goal)
		rospy.loginfo(self.cur_goal)

	def read_state(self,msg):
		self.cur_state = msg.pos;
		self.cur_node = [self.cur_state.x,self.cur_state.y, self.cur_state.z, 0, 0 ]
		self.path_nodes_list.append(self.cur_node)
		rospy.loginfo(self.cur_state)

	def read_map(self,pointClouds):
		self.map_points.points = pointClouds.points
		self.occupancy_sts.values =pointClouds.channels[0].values	
		rospy.loginfo(len(pointClouds))

	def select_move(self,cur_node, node1, node2):
		#node1 is straight and node2 is diagonal to the current node
		if calc_cost_btw_nodes(cur_node, node1) < calc_cost_btw_nodes(cur_node,	node2):
			return 	self.STRAIGHT
		else:
			return self.DIAGONAL		

	def calc_dist_btw_nodes(self,node1, node2, typ):
		if typ == self.MANHATTAN_DIST:
			dist = math.abs(node2[0] -node1[0]) + math.abs(node2[1] - node1[1]) + math.abs(node2[2] - node1[2])
		else:
			dist = (math.abs(node2[0] -node1[0])**2) + (math.abs(node2[1] - node1[1])**2) + (math.abs(node2[2] - node1[2])**2)
		return dist
	
	def calc_cost_btw_nodes(self,node1, node2):
		if node2[3] == 0: #Cost of empty cell = cur_node cost + dist between the two
			return node1[4] + calc_dist_btw_nodes(node1, node2, self.MANHATTAN_DIST)
		else: 	#Cost of occupied cell is infinity
			return self.INFINITE_COST

	def get_occupancy_sts(self,node):
		node_idx = 0
		occ_sts = SELF.OCCUPIED #For nodes not in the map, the status is OCCUPIED
		for point in self.map_points.points:
			if point == node[0:2]:
				occ_sts = self.occupancy_sts.values[node_idx] 
		return occ_sts
	
	def get_neighbours(self):
		node_c = self.cur_node; 
		d = math.floor((self.GRID_WIN_WIDTH - 1)/2)
		#Select the next probable state
		if self.move == self.STRAIGHT: #Straight Movements
			if get_occupancy_sts(node_c + [1,0,0,0,0]) == self.NOT_OCCUPIED: # forward
				node_s = node_c + [1,0,0,0,0]
			elif get_occupancy_sts(node_c + [0,1,0,0,0]) == self.NOT_OCCUPIED: #left
				node_s = node_c + [0,1,0,0,0]
			elif get_occupancy_sts(node_c + [0,-1,0,0,0]) == self.NOT_OCCUPIED: #right
				node_s = node_c + [0,-1,0,0,0]
			elif get_occupancy_sts(node_c + [0,0,1,0,0]) == self.NOT_OCCUPIED:  #up
				node_s = node_c + [0,0,1,0,0]
			else:	#down
				node_s = node_c + [0,0,-1,0,0]
		else: #Diagonal Movements
			if get_occupancy_sts(node_c + [1,1,0,0,0]) == self.NOT_OCCUPIED: # forward left
				node_s = node_c + [1,1,0,0,0]
			elif get_occupancy_sts(node_c + [1,-1,0,0,0]) == self.NOT_OCCUPIED: #forward right
				node_s = node_c + [1,-1,0,0,0]
			elif get_occupancy_sts(node_c + [1,1,1,0,0]) == self.NOT_OCCUPIED: #forward left up
				node_s = node_c + [1,1,1,0,0]
			elif get_occupancy_sts(node_c + [1,1,-1,0,0]) == self.NOT_OCCUPIED: #forward left down
				node_s = node_c + [1,1,-1,0,0]
			elif get_occupancy_sts(node_c + [1,-1,1,0,0]) == self.NOT_OCCUPIED: #forward right up
				node_s = node_c + [1,-1,1,0,0]
			elif get_occupancy_sts(node_c + [1,-1,-1,0,0]) == self.NOT_OCCUPIED: #forward right down
				node_s = node_c + [1,-1,-1,0,0]
			elif get_occupancy_sts(node_c + [1,0,1,0,0]) == self.NOT_OCCUPIED: #forward up
				node_s = node_c + [1,0,1,0,0]
			elif get_occupancy_sts(node_c + [1,0,-1,0,0]) == self.NOT_OCCUPIED: #forward down
				node_s = node_c + [1,0,-1,0,0]
			elif get_occupancy_sts(node_c + [0,1,1,0,0]) == self.NOT_OCCUPIED: #left up
				node_s = node_c + [0,1,1,0,0]
			elif get_occupancy_sts(node_c + [0,1,-1,0,0]) == self.NOT_OCCUPIED: #left down
				node_s = node_c + [0,1,-1,0,0]
			elif get_occupancy_sts(node_c + [0,-1,1,0,0]) == self.NOT_OCCUPIED: #right up
				node_s = node_c + [0,-1,1,0,0]
			else:  #right down
				node_s = node_c + [0,-1,-1,0,0]
		
    		#Add the next probable state to the neighbours list and prune nodes around it		
  		neighbours = node_s
		for k in range(-d,d):
			for j in range(-d,d):
				for i in range(0,d):
					node_i = node_c[0:2] + [i,j,k]
					node_i.append(get_occupancy_sts(node_i))
					if self.get_occupancy_sts(node_i) == self.NOT_OCCUPIED:
						if self.calc_dist_btw_nodes(node_c , node_i) > (self.calc_dist_btw_nodes(node_c, node_s) + self.calc_dist_btw_nodes(node_s, node_i)): #Pruning Cells
							neighbours.append(node_i.append(self.calc_dist_btw_nodes(node_s, node_i)))
		
		
		node_d = self.cur_node + [GRID_WIN_WIDTH,GRID_WIN_WIDTH,0,0,0]
		

	def generate_global_path(self):
		node_pose = Pose()
		global_nodes_path = Path()
		for path_node in self.path_nodes_list:			
			node_pose.position.x = path_node[0];
			node_pose.position.y = path_node[1];
			node_pose.position.z = path_node[2];
			node_pose.orientation.x = 0;
			node_pose.orientation.y = 0;
			node_pose.orientation.z = 0;
			node_pose.orientation.w = 0;	
			global_nodes_path.poses.append(node_pose);
		return global_nodes_path		

	def spin(self):		
		self.global_path = self.generate_global_path()
		self.global_path_pub.publish(self.global_path)
		rospy.spin()

if __name__ == '__main__':
	rospy.init_node('globalPlanner')
	globalPlanner_o = globalPlanner()
	globalPlanner_o.spin()
