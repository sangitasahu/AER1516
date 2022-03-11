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


   
ros::Publisher pubProb;
//tf::StampedTransform transform;

void octomap_binary_callback(const octomap_msgs::OctomapConstPtr& octomap_msg)
{
    octomap::AbstractOcTree* tree = octomap_msgs::binaryMsgToMap(* octomap_msg);

    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);

    octomap::point3d  occupancyPoints3d; 
    std::vector<_Float32> occupancyProb;
    //tf::Vector3 vectorPoints3d;
    geometry_msgs::Point32 point3d;
    sensor_msgs::ChannelFloat32 occupancyChannel;
    std::vector<geometry_msgs::Point32> points3d;
    std::vector<sensor_msgs::ChannelFloat32> occupancyChannels;
    
    for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it!= end; ++it)
    {
        occupancyPoints3d = it.getCoordinate();
        //vectorPoints3d = {occupancyPoints3d.x(),occupancyPoints3d.y(), occupancyPoints3d.z()}; 

        //tf::Vector3 transformedPoints3d = transform.operator()(vectorPoints3d);

        /*point3d.x = transformedPoints3d[0];
        point3d.y = transformedPoints3d[1];
        point3d.z = transformedPoints3d[2];*/

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
    occupancy_info.header.frame_id = "SQ01s/camera";
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
    ros::NodeHandle occupancyProbability; 
    pubProb = occupancyProbability.advertise<sensor_msgs::PointCloud>("probability_publisher",1000);
    //tf::TransformListener listner;
    /*
    while (n.ok()){
        try{
            listner.waitForTransform("world","SQ01s/camera",ros::Time(0),ros::Duration(4));
            listner.lookupTransform("world","SQ01s/camera",ros::Time(0),transform);
        }
        catch(tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
        }

    }
    */
    ros::spin();    
}