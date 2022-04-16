import numpy as np
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Header
import rospy

def get_obs(pcloud,channel):
    if channel ==[]:
        obs_ = np.array(pcloud)
        return obs_
    obs_probability = 0.5
    obs_ = np.array(pcloud)[np.argwhere(np.array(channel)>obs_probability).T[0],:]
    return obs_

def split_obs(pcloud,channel): #function to split grid from mapper into occupied and unknown
    if channel !=[]:
        occu_msg = PointCloud(header = Header(stamp = rospy.Time.now(),frame_id = "vicon"))
        unkn_msg = PointCloud(header = Header(stamp = rospy.Time.now(),frame_id = "vicon"))
        for i, state in enumerate(channel):
            p = pcloud[i]
            if state > 0.5:
                occu_msg.points.append(Point32(x=p[0],y=p[1],z=p[2]))
                continue
            if state < -0.5:
                unkn_msg.points.append(Point32(x=p[0],y=p[1],z=p[2]))
                continue
            else: #state is 0
                continue
    else:
        occu_msg = PointCloud(header = Header(stamp = rospy.Time.now(),frame_id = "vicon"))
        unkn_msg = PointCloud(header = Header(stamp = rospy.Time.now(),frame_id = "vicon"))
    return occu_msg, unkn_msg


        

