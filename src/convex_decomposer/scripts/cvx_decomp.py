#!/usr/bin/env python2

"""
ROS Node for convex decomposition of the space around a segment defined by p1 and p2. 

This ROS node subscribes to the following topics:
/probability_publisher
/global_plan

This ROS node publishes to the following topics:
/CvxDecomp
/ellipsoid_disp'
/polyhedra_disp
*disp is used to display to RVIZ visualizations

"""
# Import ROS libraries
import sys
import roslib
import rospy
# Import classes
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud
from convex_decomposer.msg import Polyhedron , CvxDecomp
from shape_msgs.msg import Plane
import hyperplane as hp
import ellipsoid as E
import point_cloud_processor as pcp
import plane_utils
from decomp_ros_msgs.msg import Polyhedron as ph_display
from decomp_ros_msgs.msg import PolyhedronArray as pharray_display
from decomp_ros_msgs.msg import Ellipsoid as E_display
from decomp_ros_msgs.msg import EllipsoidArray as Earray_display

#Define the convex decomposition class
class cvx_decomp(object):
    """
    cvx_decomp is used to perform a convex decomposition of the free space around obstacles. The setup is an implementation of
    the : MRSL Decomputil Library v1.0 :https://github.com/sikang/DecompUtil : http://ieeexplore.ieee.org/document/7839930/
    """
    def __init__(self):
        # Declare Publishers and Subscibers
        self.point_cloud = '/probability_publisher'
        self.path = '/global_plan'
        self.point_cloud_sub = rospy.Subscriber(self.point_cloud,PointCloud,self.point_cloud_proc)
        self.path_sub = rospy.Subscriber(self.path,Path,self.path_processor)
        self.pub_CvxDecomp = rospy.Publisher('/CvxDecomp',CvxDecomp,queue_size=32)
        
        #Publishers to display polygon and ellipsoids
        self.pub_Ph_display = rospy.Publisher('/polyhedra_disp',pharray_display,queue_size=32) 
        self.pub_E_display = rospy.Publisher('/ellipsoid_disp',Earray_display,queue_size=32) 

        # Select trajectory type
        # Initialize messages
        self.polyhedron = Polyhedron()
        self.cvx_polyhedra = CvxDecomp()

        self.ph_display_msg = pharray_display()
        self.E_display_msg = Earray_display()

        #self.obs_cloud = PointCloud() #Define type!
        self.obs_cloud = [] #Change to pc if necessary
        #self.obs_cloud = pcp.get_obs([[1,1,0],[2,2,0],[-2,1,0],[5,5,0],[0,14,0]],[])
        self.p = [] # placeholder for holding the end points of the segment in question
        #self.p = [[0,0,1],[0,13,1]]#,[4,4,1]] #Only to test
        self.path_list = [] #Placeholder for the path poses
        #############################################
        self.n_int_max = 6 #Does not do anything for now : plan to use it to limit the number of hyperplanes of the polyhedron
        self.offset_x = 0.1 #Offset of the ellipsoid around the segment
        self.drone_radius = 0.1 # Radius of the drone for obstacle inflation 
        self.bbox = [2,2,1]  # Local bounding box of the convex decomposer
        #############################################
        # Set the decomposer frequency
        self.loop_frequency = 10
        # Run this ROS node at the loop frequency
        self.timers = rospy.Timer(rospy.Duration(1.0 / self.loop_frequency), self.decompose_points)
        # Keep time for differentiation and integration within the controller
        self.old_time = rospy.get_time()
        
    """Function to store the current trajectory msg to be passed to the Position Controller"""
    def point_cloud_proc(self,msg):
        #split the pointcloud to free and occupied points
        cloud = msg.points
        cloud_list = [[point.x,point.y,point.z] for point in cloud]
        cloud_channel = msg.channels[0].values
        self.obs_cloud = pcp.get_obs(cloud_list,cloud_channel)
        #self.obs_cloud = hp.obs_temp()
        #print(self.obs_cloud)
        #self.obs_cloud = pcp.get_obs([[1,1,0],[2,2,0],[-2,1,0],[5,5,0],[0,14,0]],[])

    def path_processor(self,msg):
        self.path_list = msg.poses
        self.p = [[point.pose.position.x, point.pose.position.y, point.pose.position.z] for point in self.path_list]
        if len(self.p)==1:
            print("Segment cannot be formed since only one point was obtained")
        #self.p = [[0,0,1],[0,13,1]]#,[4,4,1]] #Only to test
    
    """Function called when timer matures"""
    def decompose_points(self,event):
        # Determine the time step 
        current_time = rospy.get_time()
        dt = current_time - self.old_time
        #Set header_info for the publish message:
        header_info = Header()
        header_info.stamp = rospy.Time.now()
        header_info.frame_id = "vicon"
        #Start by finding one polyhedron per segment
        poly_pack = []
        elliparray_disp_pack = []
        polyarray_disp_pack = []
        for i in range(0,len(self.p)-1):
            #Start point for the segment
            p1 = self.p[i]
            #End point for the segment
            p2 = self.p[i+1]
            #Find a bounding polyhedron based on self.bbox param.
            bbox_polyhedron = hp.add_local_bbox(p1,p2,self.bbox)
            obs = hp.set_obs_Vs(self.obs_cloud,bbox_polyhedron)            
            #Find an ellipsoid around the segment
            C,d = E.find_ellipsoid(p1,p2,self.offset_x,obs,self.drone_radius)
            #Get the hyperplanes at the extremeties and eliminate the spaces
            cvx_planes = hp.get_hyperplanes(obs,C,d)
            #Add the bounding box and cvx_decomp_planes to obtain inner polyhedrons
            complex_polyhedron = cvx_planes+bbox_polyhedron
            #Simplify the plyhedrons to get inner polyhedrons only 
            # Plan is to simply eliminate planes that are parallel by distance to path segment in question.
            required_polyhedron = plane_utils.eliminate_redundant_planes(complex_polyhedron)
            #print(required_polyhedron)
            vertices = plane_utils.calculate_vertices(complex_polyhedron)
            ###Pending work<<< 
            #FIND A FOOL PROOF WAY TO LIMIT TOTAL NUMBER OF PLANES IN A POLYHEDRA?
            #convert polyhedrons from point normal to standard form >> ax+by+cz=d
            plane_pack = []
            polyhedron_ = Polyhedron()
            ellip_disp_msg = E_display()
            poly_disp_msg = ph_display()
            #Append the ellipsoid for display
            ellip_disp_msg.E = C.ravel().tolist()
            ellip_disp_msg.d = d.tolist()
            elliparray_disp_pack.append(ellip_disp_msg)


            normals_pack = []
            points_pack = []
            for j,pln in enumerate(required_polyhedron):
                plane = hp.get_standard_form(pln)
                plane_msg = Plane()
                plane_msg.coef = plane
                plane_pack.append(plane_msg)
                #form point msg and normal msgs
                point_msg = Point()
                normal_msg = Point()
                point_msg.x = pln[0][0]
                point_msg.y = pln[0][1]
                point_msg.z = pln[0][2]
                normal_msg.x = pln[1][0]
                normal_msg.y = pln[1][1]
                normal_msg.z = pln[1][2]
                #create the message list
                normals_pack.append(normal_msg)
                points_pack.append(point_msg)

            poly_disp_msg.points = points_pack
            poly_disp_msg.normals = normals_pack
            polyhedron_.planes= plane_pack
            #Make a pack for the array
            poly_pack.append(polyhedron_)        
            polyarray_disp_pack.append(poly_disp_msg)
        self.cvx_polyhedra.polyhedra = poly_pack
        self.cvx_polyhedra.header = header_info
        self.pub_CvxDecomp.publish(self.cvx_polyhedra)
        #publish to display on rviz
        self.ph_display_msg.polyhedrons = polyarray_disp_pack
        self.E_display_msg.ellipsoids = elliparray_disp_pack
        self.E_display_msg.header = header_info
        self.ph_display_msg.header = header_info
        self.pub_E_display.publish(self.E_display_msg)
        self.pub_Ph_display.publish(self.ph_display_msg)
        #plotter(p1,p2,obs_init,elp_sample (C,d,100))###PLOTTER
        #plotter_poly(globalpoly)
if __name__ == '__main__':
    # Code to create cvx_decomp
    rospy.init_node('cvx_decomp',disable_signals=True)

    cvx_decomp()
    rospy.spin()