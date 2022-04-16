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
from sensor_msgs.msg import PointCloud
import numpy as np
import point_cloud_processor as pcp

#Instantiates objects of classes  Elephant and Tiger

class repub_class(object):
    """ROS interface for controlling the Parrot """
    """Constructor to initialize the Elephant class"""
    def __init__(self):
        # Declare Publisher and Subsciber
        
        self.unkn_ = '/unknown_grid'
        self.unkn_pub = rospy.Publisher(self.unkn_,PointCloud,queue_size=1) 
        self.occupied_ = '/occup_grid'
        self.occupied_pub = rospy.Publisher(self.occupied_,PointCloud,queue_size=1) 
        self.pcl_original = '/grid_publisher'
        self.pcl_sub = rospy.Subscriber(self.pcl_original,PointCloud,callback=self.pcl_processor)
    
    def pcl_processor(self,msg):
        cloud = msg.points
        cloud_list = [[point.x,point.y,point.z] for point in cloud]
        cloud_channel = msg.channels[0].values

        occupied_msg,unkn_msg = pcp.split_obs(cloud_list,cloud_channel)

        self.occupied_pub.publish(occupied_msg)
        self.unkn_pub.publish(unkn_msg)

if __name__ == '__main__':
    # Code to create a repub node
    rospy.init_node('repub',disable_signals=True)
    repub_class()
    rospy.spin()