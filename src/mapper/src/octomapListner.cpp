#include "ros/ros.h"
#include "octomap_msgs/Octomap.h"
#include <octomap/AbstractOcTree.h>
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
   
ros::Publisher pubProb;
ros::Publisher pubGrid;
ros::Publisher pubPoints;
snapstack_msgs::State quad_state;
geometry_msgs::TransformStamped tf_cam2world;
geometry_msgs::Vector3 quad_pos;
octomap::point3d  occupancyPoints3d; 


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

void octomap_binary_callback(const octomap_msgs::OctomapConstPtr& octomap_msg)
{

    quad_pos = quad_state.pos;
    
    //octomap::AbstractOcTree* tree = octomap_msgs::binaryMsgToMap(* octomap_msg);

    //octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);

    std::shared_ptr<octomap::OcTree> octree = std::shared_ptr<octomap::OcTree> (dynamic_cast<octomap::OcTree*> (octomap_msgs::msgToMap(* octomap_msg)));

    int z_scale_factor = 6;
    float bbx_range = 18.;
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
    std::vector<sensor_msgs::ChannelFloat32> occupancyChannels;

    int tree_index = 0;
    for(octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(min_bbx,max_bbx), end = octree->end_leafs_bbx(); it!= end; ++it)
    {
        tree_index++;

        occupancyPoints3d = it.getCoordinate();

        point3d.x = occupancyPoints3d.x();
        point3d.y = occupancyPoints3d.y();
        point3d.z = occupancyPoints3d.z();

        point3d.x = round_val(point3d.x);
        point3d.y = round_val(point3d.y);
        point3d.z = round_val(point3d.z);

        //box limits

        float frontLim = point3d.x + resolution;
        float rearLim = point3d.x - resolution;
        float leftLim = point3d.y + resolution;
        float rightLim = point3d.y - resolution;
        float upperLim = point3d.z + resolution;
        float lowerLim = point3d.z - resolution;



        if(tree_index==1){
            offset3d.x = quad_pos.x;
            offset3d.y = quad_pos.y;
            offset3d.z = quad_pos.z;
            grids3d = grid_point_gen(offset3d,min_bbx,max_bbx,resolution);
            gridOccupancy.resize(grids3d.size(), -1.);
        }

        //ROS_INFO("grid");
        for(int grid_index =0; grid_index<grids3d.size(); grid_index++){   
                        
            //if(abs(grids3d[grid_index].x-point3d.x)<1e-3 && abs(grids3d[grid_index].y-point3d.y)<1e-3 && abs(grids3d[grid_index].z-point3d.z)<1e-3){
            if (grids3d[grid_index].x>=rearLim && grids3d[grid_index].x<=frontLim && grids3d[grid_index].y>=rightLim && grids3d[grid_index].y<=leftLim && grids3d[grid_index].z>=lowerLim && grids3d[grid_index].z<=upperLim){
                if(it->getOccupancy()>0.5){
                    gridOccupancy[grid_index] = 1.;
                }
                else{
                    gridOccupancy[grid_index] = 0.;
                }
                
            }
        }

        points3d.push_back(point3d);

        occupancyProb.push_back(it->getOccupancy());

    }
    

    occupancyChannel.name = "occupancy probability";
    occupancyChannel.values = occupancyProb;
    occupancyChannels.push_back(occupancyChannel);    

    gridChannel.name = "grid occupancy";
    gridChannel.values = gridOccupancy;
    gridChannels.push_back(gridChannel);


    sensor_msgs::PointCloud occupancy_info;
    occupancy_info.header.frame_id = "vicon";
    occupancy_info.header.stamp = ros::Time(0);
    occupancy_info.points = points3d;
    occupancy_info.channels = occupancyChannels;
    
    sensor_msgs::PointCloud grid_info;
    grid_info.header.frame_id = "vicon";
    grid_info.header.stamp = ros::Time(0);   
    grid_info.points = grids3d;
    grid_info.channels = gridChannels;


    pubProb.publish(occupancy_info); 
    pubGrid.publish(grid_info);

    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomapListener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("octomap_binary", 1000, octomap_binary_callback);
    ros::NodeHandle state;
    ros::Subscriber sub_state = state.subscribe("/SQ01s/state", 10, state_callback);  
    ros::NodeHandle occupancyProbability; 
    ros::NodeHandle gridNode;
    ros::NodeHandle points;
    pubProb = occupancyProbability.advertise<sensor_msgs::PointCloud>("probability_publisher",1000);
    pubGrid = gridNode.advertise<sensor_msgs::PointCloud>("grid_publisher",1000);

    ros::spin();    
}
