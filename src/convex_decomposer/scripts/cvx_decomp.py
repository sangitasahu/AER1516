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
#from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import Header
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud
from convex_decomposer.msg import Polyhedron , CvxDecomp
from shape_msgs.msg import Plane
import hyperplane as hp
import ellipsoid as E
import point_cloud_processor as pcp

#Instantiates objects of classes ROSDesiredPositionGenerator and PositionController

class cvx_decomp(object):
    """ROS interface for controlling the Parrot ARDrone in the Vicon Lab."""
    """Constructor to initialize the ROSControllerNode class"""
    def __init__(self):
        # Declare Publisher and Subsciber
        self.point_cloud = '/probability_publisher'
        self.point_cloud_sub = rospy.Subscriber(self.point_cloud,PointCloud,self.point_cloud_proc)
        self.path = '/global_plan'
        self.path_sub = rospy.Subscriber(self.path,Path,self.path_processor)
        self.pub_CvxDecomp = rospy.Publisher('/CvxDecomp',CvxDecomp,queue_size=32)

        # Select trajectory type
        # Initialize messages
        self.polyhedron = Polyhedron()
        self.cvx_polyhedra = CvxDecomp()
        #self.obs_cloud = PointCloud() #Define type!
        self.obs_cloud = [] #Change to pc if necessary
        #self.obs_cloud = pcp.get_obs([1,1,0],[2,2,0],[-2,1,0],[5,5,0],[0,14,0]])
        self.p = [] # from point
        #self.p = [[0,0,0],[0,13,0],[4,13,0]] #Only to test

        self.n_int_max = 6
        self.offset_x = 0
        self.drone_radius = 0
        self.bbox = [2,2,1]
        self.path_list = []
        # Set the controller frequency
        self.loop_frequency = 1.0
        # Run this ROS node at the loop frequency
        self.pub_vel_cmd_timer = rospy.Timer(rospy.Duration(1.0 / self.loop_frequency), self.decompose_points)
        
        # Keep time for differentiation and integration within the controller
        self.old_time = rospy.get_time()
        
    """Function to store the current trajectory msg to be passed to the Position Controller"""
    def point_cloud_proc(self,msg):
        #split the pointcloud to free and occupied points
        cloud = msg.points
        cloud_list = [[point.x,point.y,point.z] for point in cloud]
        cloud_channel = msg.channels
        self.obs_cloud = pcp.get_obs(cloud_list,cloud_channel)

    def path_processor(self,msg):
        self.path_list = msg.poses
        self.p = [[point.pose.position.x, point.pose.position.y, point.pose.position.z] for point in self.path_list]


    """Function called when timer matures"""
    def decompose_points(self,event):

        # Determine the time step
        current_time = rospy.get_time()
        dt = current_time - self.old_time
        obs = self.obs_cloud
        #Set header_info for the Publish Message:
        header_info = Header()
        header_info.stamp = rospy.Time.now()
        header_info.frame_id = "cvx"
        #Start by finding one polyhedron per segment
        poly_pack = []
        for i in range(0,len(self.p)-1):
            #Start point for the segment
            p1 = self.p[i] 
            #End point for the segment
            p2 = self.p[i+1] 
            #Find a bounding polyhedron based on self.bbox param.
            bbox_polyhedron = hp.add_local_bbox(p1,p2,self.bbox)
            #Find an ellipsoid around the segment
            C,d = E.find_ellipsoid(p1,p2,self.offset_x,obs,self.drone_radius)
            #Get the hyperplanes at the extremeties and eliminate the spaces
            cvx_planes = hp.get_hyperplanes(obs,C,d)
            #Add the bounding box and cvx_decomp_planes to obtain inner polyhedrons
            complex_polyhedron = cvx_planes+bbox_polyhedron

            #Simplify the plyhedrons to get inner polyhedrons only
            ###Pending work<<< 
            
            #FIND A WAY TO LIMIT TOTAL NUMBER OF PLANES IN A POLYHEDRA

            required_polyhedron = complex_polyhedron #Change this after completing above
            #convert polyhedrons from point normal to standard form >> ax+by+cz=d
            plane_pack = []
            polyhedron_ = Polyhedron()
            for j,pln in enumerate(required_polyhedron):
                plane = hp.get_standard_form(pln)
                plane_msg = Plane()
                plane_msg.coef = plane
                #print(j," J ",plane_msg,": plane")
                plane_pack.append(plane_msg)
            polyhedron_.planes= plane_pack
            poly_pack.append(polyhedron_)        
        self.cvx_polyhedra.polyhedra = poly_pack
        self.cvx_polyhedra.header = header_info
        self.pub_CvxDecomp.publish(self.cvx_polyhedra)
        #plotter(p1,p2,obs_init,elp_sample (C,d,100))###PLOTTER
        #plotter_poly(globalpoly)
if __name__ == '__main__':
    # Code to create cvx_decomp
    rospy.init_node('cvx_decomp',disable_signals=True)
    cvx_decomp()
    rospy.spin()