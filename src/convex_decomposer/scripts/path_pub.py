#!/usr/bin/env python2

"""
ROS Node for convex decomposition of the space around a segment defined by p1 and p2. 

This ROS node subscribes to the following topics:
/vicon/ARDroneCarre/ARDroneCarre

This ROS node publishes to the following topics:
/cmd_vel_RHC

"""

# Import ROS libraries
import sys
import roslib
import rospy
# Import classes
#from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped,Point,Pose, PointStamped
from std_msgs.msg import Header
from nav_msgs.msg import Path
from snapstack_msgs.msg import Goal
import numpy as np
import time
#Instantiates objects of classes ROSDesiredPositionGenerator and PositionController

class pub_path_class(object):
    """ROS interface for controlling the Parrot ARDrone in the Vicon Lab."""
    """Constructor to initialize the ROSControllerNode class"""
    def __init__(self):
        #self.pth = [[13.5,11.5,1],[15,12.5,1],[16,13,1]]
        self.pth = [[1,2,1],[2,2,1],[3,2,1],[4,3,1]]

        # Declare Publisher and Subsciber
        
        self.path = '/global_plan'
        self.path_pub = rospy.Publisher(self.path,Path,queue_size=1) 
        self.droneloc = '/SQ01s/goal'
        self.drone_loc_pub = rospy.Publisher(self.droneloc,Goal,queue_size=1) 
        #self.clicked_point_topic = 'clicked_point'
        #self.clicked_point_sub = rospy.Subscriber(self.clicked_point_topic,PointStamped,callback=self.clicked_point_sub_callback)
        self.clicked_point_topic = '/move_base_simple/goal'
        self.clicked_point_sub = rospy.Subscriber(self.clicked_point_topic,PoseStamped,callback=self.clicked_point_sub_callback)

        

        self.p = Path()
        self.received_point = False
        # Set the controller frequency
        self.loop_frequency = 4.0
        # Run this ROS node at the loop frequency
        self.timers = rospy.Timer(rospy.Duration(1.0 / self.loop_frequency), self.pub_path_def)
    
    def clicked_point_sub_callback(self,msg):
        self.received_point = True
        self.clicked_point = msg
    
    """Function called when timer matures"""
    def pub_path_def(self,event):
        #Set header_info for the Publish Message:
        header_info = Header()
        header_info.stamp = rospy.Time.now()
        header_info.frame_id = "vicon"
        pose_pack= []

        #if self.received_point:
            # Update global goal with current clicked point value, held at target flight Z level
            #point = [self.clicked_point.pose.position.x,self.clicked_point.pose.position.y,1]
            #self.received_point = False

#            if len(self.pth)>=2:
 #               self.pth = [self.pth[1]]
                #self.pth.append(point)
            #else:
             #   self.pth.append(point)

        #posestd = Path()
        print(self.pth)
        for pt in self.pth:
            #posestamped = PoseStamped()
            #posestamped.pose.position.x=pt[0]
            #posestamped.pose.position.y=pt[1]
            #posestamped.pose.position.z=pt[2]
            #posestd.poses.append(posestamped)
            msg = Goal()
            msg.p.x = pt[0]
            msg.p.y = pt[1]
            msg.p.z = pt[2]
            self.drone_loc_pub.publish(msg)
            time.sleep(2)

        #print(posestd)
        #posestd.header = header_info
        #self.path_pub.publish(posestd)
        #loc_ = self.pth[0]
        #msg = Goal()
        #msg.p.x = self.pth[0][0]
        #msg.p.y = self.pth[0][1]
        #msg.p.z = self.pth[0][2]
        #yawmsg = np.arctan2(self.pth[1][1]-self.pth[0][1],self.pth[1][0]-self.pth[0][0])
        #msg.yaw = yawmsg
        #self.drone_loc_pub.publish(msg)
if __name__ == '__main__':
    # Code to create cvx_decomp
    rospy.init_node('path_pub',disable_signals=True)
    pub_path_class()
    rospy.spin()