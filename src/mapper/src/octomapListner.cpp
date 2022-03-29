#include "ros/ros.h"
#include "octomap_msgs/Octomap.h"
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <vector>
#include "snapstack_msgs/State.h"
#include <string>
   
ros::Publisher pubProb;
ros::Publisher pubGrid;
ros::Publisher pubPoints;
snapstack_msgs::State quad_state;
geometry_msgs::Vector3 quad_pos;
octomap::point3d  occupancyPoints3d; 


void state_callback(const snapstack_msgs::State& state_msg)
{
    quad_state = state_msg;
}

void octomap_binary_callback(const octomap_msgs::OctomapConstPtr& octomap_msg)
{

    quad_pos = quad_state.pos;
    
    //octomap::AbstractOcTree* tree = octomap_msgs::binaryMsgToMap(* octomap_msg);

    //octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);

    std::shared_ptr<octomap::OcTree> octree = std::shared_ptr<octomap::OcTree> (dynamic_cast<octomap::OcTree*> (octomap_msgs::msgToMap(* octomap_msg)));

    int bbx_upper = 2;
    int bbx_lower = 0;
    float resolution = 0.1;
    int bsize = static_cast<int>((bbx_upper-bbx_lower)/resolution);
    octomap::point3d  min_bbx;
    min_bbx.x() = quad_pos.x+bbx_lower;
    min_bbx.y() = quad_pos.y+bbx_lower;
    min_bbx.z() = quad_pos.z+bbx_lower;
    octomap::point3d  max_bbx;
    max_bbx.x() = quad_pos.x+bbx_upper;
    max_bbx.y() = quad_pos.y+bbx_upper;
    max_bbx.z() = quad_pos.z+bbx_upper;



    std::vector<_Float32> occupancyProb;
      
    geometry_msgs::Point32 point3d;
    sensor_msgs::ChannelFloat32 occupancyChannel;
    std::vector<geometry_msgs::Point32> points3d;
    std::vector<sensor_msgs::ChannelFloat32> occupancyChannels;

    std::vector<std::vector<std::vector<int>>> grid(bsize,std::vector<std::vector<int>>(bsize,std::vector<int>(bsize)));


    for(octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(min_bbx,max_bbx), end = octree->end_leafs_bbx(); it!= end; ++it)
    {
        occupancyPoints3d = it.getCoordinate();

        point3d.x = occupancyPoints3d.x();
        point3d.y = occupancyPoints3d.y();
        point3d.z = occupancyPoints3d.z();

        int x_index = int((occupancyPoints3d.x()-(quad_pos.x+0.05))/resolution);
        int y_index = int((occupancyPoints3d.y()-(quad_pos.y+0.05))/resolution);
        int z_index = int((occupancyPoints3d.z()-(quad_pos.z+0.05))/resolution);


        if(it->getOccupancy()>0.5){
            grid[x_index][y_index][z_index]=1;
        }

        points3d.push_back(point3d);

        occupancyProb.push_back(it->getOccupancy());

    }
    
    occupancyChannel.name = "occupancy probability";
    occupancyChannel.values = occupancyProb;

    occupancyChannels.push_back(occupancyChannel);


    sensor_msgs::PointCloud occupancy_info;
    occupancy_info.header.frame_id = "vicon";
    occupancy_info.header.stamp = ros::Time(0);
    occupancy_info.points = points3d;
    occupancy_info.channels = occupancyChannels;

    pubProb.publish(occupancy_info); 

    // 3d occupancy grid
    std_msgs::Int32MultiArray dat;

    dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dat.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dat.layout.dim[0].label = "x dimension";
    dat.layout.dim[1].label = "y dimension";
    dat.layout.dim[2].label = "z dimension";
    dat.layout.dim[0].size = bsize;
    dat.layout.dim[1].size = bsize;
    dat.layout.dim[2].size = bsize;
    dat.layout.dim[0].stride = bsize*bsize*bsize;
    dat.layout.dim[1].stride = bsize*bsize;
    dat.layout.dim[2].stride = bsize;
    dat.layout.data_offset = 0;
    std::vector<int> vec(bsize*bsize*bsize,0);
    for(int i = 0; i<bsize; i++){
        for(int j=0; j<bsize; j++){
            for(int k =0; k<bsize; k++){
                vec[i*bsize*bsize+j*bsize+k] = grid[i][j][k];
            }
        }
    }
    dat.data = vec;

    pubGrid.publish(dat);
    
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
    pubGrid = gridNode.advertise<std_msgs::Int32MultiArray>("grid_publisher",1000);

    ros::spin();    
}