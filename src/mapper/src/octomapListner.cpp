#include "ros/ros.h"
#include "octomap_msgs/Octomap.h"
#include <octomap/AbstractOcTree.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <vector>
#include <algorithm>
#include "snapstack_msgs/State.h"
#include <string>
#include <cmath>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
   
ros::Publisher pubProb;
ros::Publisher pubGrid;
ros::Publisher pubOccup;
ros::Publisher pubOccup2;
ros::Publisher pubProb2;

snapstack_msgs::State quad_state;
geometry_msgs::TransformStamped tf_cam2world;
geometry_msgs::Vector3 quad_pos;
octomap::point3d  occupancyPoints3d; 
geometry_msgs::Vector3 trans;
tf2::Quaternion rot_tf;

void tf_callback(const tf2_msgs::TFMessage& tf_msg)
{

    geometry_msgs::Quaternion rot;
    tf_cam2world = tf_msg.transforms[0];

    if(tf_cam2world.header.frame_id.compare("vicon") == 0){
        trans = tf_cam2world.transform.translation;
        rot = tf_cam2world.transform.rotation;
        tf2::Quaternion rot_tf_inv = tf2::Quaternion(rot.x, rot.y, rot.z, rot.w);
        rot_tf = rot_tf_inv.inverse();  
    }


}

void state_callback(const snapstack_msgs::State& state_msg)
{
    quad_state = state_msg;

}


float round_val(float var){
    float value = (int)(var*100+0.5);
    return (float)value/100;
}

std::vector<geometry_msgs::Point32> grid_point_gen(geometry_msgs::Point32 offset_point, octomap::point3d  min_bbx, octomap::point3d  max_bbx, float resolution){    
    geometry_msgs::Point32 grid3d;
    std::vector<geometry_msgs::Point32> grids3d;
    for(float i = min_bbx.z(); i<=max_bbx.z(); i=i+resolution){
        for(float j = min_bbx.y(); j<=max_bbx.y(); j=j+resolution){
            for(float k=min_bbx.x(); k<=max_bbx.x(); k=k+resolution){
                grid3d.x = k;
                grid3d.y = j;
                grid3d.z = i;
                grids3d.push_back(grid3d);
            }
        
        }

    }    
    return grids3d;

}

bool camera_fov_gen(float x, float y, float z){    
    //camera cone
    bool is_fov = false;
    if (x>0. && x<=10. ){
        
        std::vector<float> camera_lower_limit{x, float(-1.73205)*x, -x};
        std::vector<float> camera_upper_limit{x, float(1.73205)*x, x};

        if (y>=camera_lower_limit[1] && y<=camera_upper_limit[1] && z>=camera_lower_limit[2] && z<=camera_upper_limit[2]){
            is_fov = true;    
        }
    }

    return is_fov;

}


void octomap_binary_callback(const octomap_msgs::OctomapConstPtr& octomap_msg)
{

    quad_pos = quad_state.pos;

    std::shared_ptr<octomap::OcTree> octree = std::shared_ptr<octomap::OcTree> (dynamic_cast<octomap::OcTree*> (octomap_msgs::msgToMap(* octomap_msg)));

    int z_scale_factor = 3;
    float bbx_range = 6.;
    float bbx_range_z = bbx_range/z_scale_factor;
    float resolution = 0.25;
    //int bsize = static_cast<int>((bbx_upper-bbx_lower)/resolution);
    octomap::point3d  min_bbx;
    min_bbx.x() = round_val(quad_pos.x)-bbx_range/2;
    min_bbx.y() = round_val(quad_pos.y)-bbx_range/2;
    min_bbx.z() = round_val(quad_pos.z)-bbx_range_z/2;
    octomap::point3d  max_bbx;
    max_bbx.x() = round_val(quad_pos.x)+bbx_range/2;
    max_bbx.y() = round_val(quad_pos.y)+bbx_range/2;
    max_bbx.z() = round_val(quad_pos.z)+bbx_range_z/2; 

    geometry_msgs::Point32 grid3d; 
    std::vector<geometry_msgs::Point32> grids3d; 
    //std::vector<geometry_msgs::Point32>::iterator gridIt;
    std::vector<_Float32> gridOccupancy;  
    sensor_msgs::ChannelFloat32 gridChannel; 
    std::vector<sensor_msgs::ChannelFloat32> gridChannels;                          


    std::vector<_Float32> occupancyProb;
      
    geometry_msgs::Point32 point3d;
    geometry_msgs::Point32 offset3d;
    sensor_msgs::ChannelFloat32 occupancyChannel;
    std::vector<geometry_msgs::Point32> points3d;
    std::vector<geometry_msgs::Point32> Occup3d;
    std::vector<sensor_msgs::ChannelFloat32> occupancyChannels;


    offset3d.x = quad_pos.x; 
    offset3d.y = quad_pos.y;
    offset3d.z = quad_pos.z;
    
    //tf2::Matrix3x3 rotMat(rot); 

    grids3d = grid_point_gen(offset3d,min_bbx,max_bbx,resolution);
    gridOccupancy.resize(grids3d.size(), -1.);

    for(int grid_index =0; grid_index<grids3d.size(); grid_index++){  

        float grid_x = grids3d[grid_index].x;
        float grid_y = grids3d[grid_index].y;
        float grid_z = grids3d[grid_index].z;

        float trans_x = grids3d[grid_index].x - trans.x;
        float trans_y = grids3d[grid_index].y - trans.y;
        float trans_z = grids3d[grid_index].z - trans.z;

        tf2::Vector3 trans_vec(trans_x, trans_y, trans_z);

        tf2::Vector3 rot_vec;
        rot_vec = tf2::quatRotate(rot_tf, trans_vec);

        float rot_x = rot_vec.x();
        float rot_y = rot_vec.y();
        float rot_z = rot_vec.z();

        bool in_fov = camera_fov_gen(rot_x, rot_y, rot_z);

        if (in_fov){

            gridOccupancy[grid_index] = 0.;


            octomap::OcTreeNode* result = octree->search(grids3d[grid_index].x, grids3d[grid_index].y, grids3d[grid_index].z);

            if(result){
                if(result->getOccupancy()>0.5){
                    gridOccupancy[grid_index] = 1.;
                    Occup3d.push_back(grids3d[grid_index]);
                }
            }
        }
        else{
            points3d.push_back(grids3d[grid_index]);
        }

    }

    gridChannel.name = "grid occupancy";
    gridChannel.values = gridOccupancy;
    gridChannels.push_back(gridChannel);


    sensor_msgs::PointCloud unknown_info;
    unknown_info.header.frame_id = "vicon";
    unknown_info.header.stamp = ros::Time(0);
    unknown_info.points = points3d;

    sensor_msgs::PointCloud occupancy_info;
    occupancy_info.header.frame_id = "vicon";
    occupancy_info.header.stamp = ros::Time(0);
    occupancy_info.points = Occup3d;

    sensor_msgs::PointCloud grid_info;
    grid_info.header.frame_id = "vicon";
    grid_info.header.stamp = ros::Time(0);   
    grid_info.points = grids3d;
    grid_info.channels = gridChannels;

    sensor_msgs::PointCloud2 unknown_info2;
    unknown_info2.header.frame_id = "vicon";
    unknown_info2.header.stamp = ros::Time(0);   
    
    sensor_msgs::PointCloud2 occupancy_info2;
    occupancy_info2.header.frame_id = "vicon";
    occupancy_info2.header.stamp = ros::Time(0);   

    sensor_msgs::convertPointCloudToPointCloud2(unknown_info,unknown_info2);
    sensor_msgs::convertPointCloudToPointCloud2(occupancy_info,occupancy_info2);

    pubProb.publish(unknown_info);
    pubOccup.publish(occupancy_info); 

    pubProb2.publish(unknown_info2);
    pubOccup2.publish(occupancy_info2); 

    pubGrid.publish(grid_info);

    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomapListener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("octomap_binary", 10, octomap_binary_callback);
    
    ros::NodeHandle state;
    ros::Subscriber sub_state = state.subscribe("/SQ01s/state", 10, state_callback);  

    ros::NodeHandle tf;
    ros::Subscriber sub_tf = tf.subscribe("/tf", 10, tf_callback);  

    ros::NodeHandle occupancyProbability; 
    ros::NodeHandle gridNode;
    ros::NodeHandle free;
    
    pubProb = occupancyProbability.advertise<sensor_msgs::PointCloud>("unknown_grid1",1);
    pubGrid = gridNode.advertise<sensor_msgs::PointCloud>("grid_publisher",1);
    pubOccup = occupancyProbability.advertise<sensor_msgs::PointCloud>("occup_grid1",1);

    pubOccup2 = occupancyProbability.advertise<sensor_msgs::PointCloud2>("occup_grid",1);
    pubProb2 = occupancyProbability.advertise<sensor_msgs::PointCloud2>("unknown_grid",1);

    ros::spin();    
}