#!/usr/bin/env python2

"""
ROS Node for convex decomposition of the space around a segment defined by p1 and p2. 

This ROS node subscribes to the following topics:
/vicon/ARDroneCarre/ARDroneCarre

This ROS node publishes to the following topics:
/cmd_vel_RHC

"""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import sys
import roslib
import rospy
# Import classes
#from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped,Point,Pose
from std_msgs.msg import Header
from nav_msgs.msg import Path

#Instantiates objects of classes ROSDesiredPositionGenerator and PositionController

class pub_path_class(object):
    """ROS interface for controlling the Parrot ARDrone in the Vicon Lab."""
    """Constructor to initialize the ROSControllerNode class"""
    def __init__(self):
        self.pth = [[7,5,1],[8,5,1]]
        # Declare Publisher and Subsciber
        
        self.path = '/global_plan'
        self.path_pub = rospy.Publisher(self.path,Path,queue_size=32) 

        self.p = Path()
        # Set the controller frequency
        self.loop_frequency = 4.0
        # Run this ROS node at the loop frequency
        self.timers = rospy.Timer(rospy.Duration(1.0 / self.loop_frequency), self.pub_path_def)
    
    """Function called when timer matures"""
    def pub_path_def(self,event):
        #Set header_info for the Publish Message:
        header_info = Header()
        header_info.stamp = rospy.Time.now()
        header_info.frame_id = "map"
        pose_pack= []
        
        
        posestd = Path()
        for pt in self.pth:
            posestamped = PoseStamped()
            posestamped.pose.position.x=pt[0]
            posestamped.pose.position.y=pt[1]
            posestamped.pose.position.z=pt[2]
            posestd.poses.append(posestamped)
        #print(posestd)
        posestd.header = header_info
        self.path_pub.publish(posestd)
if __name__ == '__main__':
    # Code to create cvx_decomp
    rospy.init_node('path_pub',disable_signals=True)
    pub_path_class()
    rospy.spin()