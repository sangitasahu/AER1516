#include "ros/ros.h"
#include "octomap_msgs/Octomap.h"
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <vector>
#include <tf/transform_listener.h>
#include "snapstack_msgs/State.h"
#include <string>
   

ros::Publisher pubProb;
ros::Publisher pubPoints;
tf::StampedTransform transform;
snapstack_msgs::State quad_state;

std::vector<std::vector<float>> sortedPointCloud(sensor_msgs::PointCloud& pointsCloud){
    std::vector<geometry_msgs::Point32> points3d = pointsCloud.points;
    std::vector<std::vector<float>> pointCloud;
    std::vector<float> occupancyData = pointsCloud.channels[0].values;
    int index = 0;
    for(auto point:points3d){
        pointCloud.push_back({point.x, point.y, point.z, occupancyData[index]});
    }
    std::sort(pointCloud.begin(), pointCloud.end());

    return pointCloud;
}

void state_callback(const snapstack_msgs::State& state_msg)
{
    quad_state = state_msg;
    geometry_msgs::Vector3 quad_pos = quad_state.pos;
}

void octomap_binary_callback(const octomap_msgs::OctomapConstPtr& octomap_msg)
{

    geometry_msgs::Vector3 quad_pos = quad_state.pos;
    
    octomap::AbstractOcTree* tree = octomap_msgs::binaryMsgToMap(* octomap_msg);

    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);

    octomap::point3d  occupancyPoints3d; 
    octomap::point3d  min_bbx;
    min_bbx.x() = quad_pos.x-2;
    min_bbx.y() = quad_pos.y-2;
    min_bbx.z() = quad_pos.z-2;
    octomap::point3d  max_bbx;
    max_bbx.x() = quad_pos.x+5.;
    max_bbx.y() = quad_pos.y+5.;
    max_bbx.z() = quad_pos.z+5.;

    std::vector<_Float32> occupancyProb;
      
    tf::Vector3 vectorPoints3d;
    geometry_msgs::Point32 point3d;
    sensor_msgs::ChannelFloat32 occupancyChannel;
    std::vector<geometry_msgs::Point32> points3d;
    std::vector<sensor_msgs::ChannelFloat32> occupancyChannels;


    for(octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(min_bbx,max_bbx), end = octree->end_leafs_bbx(); it!= end; ++it)
    {
        occupancyPoints3d = it.getCoordinate();

        point3d.x = occupancyPoints3d.x();
        point3d.y = occupancyPoints3d.y();
        point3d.z = occupancyPoints3d.z();

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

    
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "octomapListener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("octomap_binary", 1000, octomap_binary_callback);
    ros::NodeHandle state;
    ros::Subscriber sub_state = state.subscribe("/SQ01s/state", 10, state_callback);
    ros::NodeHandle occupancyProbability; 
    ros::NodeHandle points;
    pubProb = occupancyProbability.advertise<sensor_msgs::PointCloud>("probability_publisher",1000);

    ros::spin();    
}
