#!/usr/bin/env python

#
# AER 1516 Motion Planning Project
# Local Planner - Test Talker
#

# Import libraries
import rospy
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis, quaternion_multiply

# Import message types
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Vector3, Quaternion, PoseStamped, Pose
from shape_msgs.msg import Plane
from std_msgs.msg import Float64, Header
from local_planner.msg import CvxDecomp, Polyhedron

class local_planner_test_talker:
    def __init__(self):
        # Publishers
        self.glob_plan_topic = 'global_plan'
        self.glob_plan_pub = rospy.Publisher(self.glob_plan_topic,Path,queue_size=10)
        self.cvx_decomp_topic = 'cvx_decomp'
        self.cvx_decomp_pub = rospy.Publisher(self.cvx_decomp_topic,CvxDecomp,queue_size=10)

        # Parameters
        self.planes = np.array([[0,1,2,3],
                                [4,5,6,7]])

        self.positions = np.array([[0,0,0],
                                [2,3.5,12],
                                [10,15,20]])

        # Published values
        self.global_plan = Path()
        self.cvx_decomp = CvxDecomp()

        pose_list = []
        for i in range(self.positions.shape[0]):
            pose_new = PoseStamped(pose=Pose(position=Point(x = self.positions[i,0],
                                                    y = self.positions[i,1],
                                                    z = self.positions[i,2])))
            pose_list.append(pose_new)
        self.global_plan.poses = pose_list

        plane_list = []
        for i in range(self.planes.shape[0]):
            plane_list.append(Plane(coef=self.planes[i,:].tolist()))

        self.cvx_decomp = CvxDecomp(polyhedra = [Polyhedron(planes=plane_list)])

        self.loop_rate = 10 # Hz
        self.report_rate = 50
        self.rate = rospy.Rate(self.loop_rate)

    def talker(self):
        counter = 0
        while not rospy.is_shutdown():
            self.global_plan.header.stamp = rospy.get_rostime()
            self.glob_plan_pub.publish(self.global_plan)
            self.cvx_decomp_pub.publish(self.cvx_decomp)
            if counter%100 == 0:
                rospy.loginfo("Published global plan/convex decomp, time = %d",rospy.get_rostime().to_sec())
            counter+=1
            self.rate.sleep()


if __name__ == '__main__' :
    try:
        # Initialize node
        rospy.init_node('local_planner_test_talker')
        test_talker = local_planner_test_talker()
        test_talker.talker()
    except rospy.ROSInterruptException:
        pass