/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"

#include "ros/ros.h"

#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

//#include <atomic>

#include <Eigen/Dense>
#include <snapstack_msgs/State.h>
#include <snapstack_msgs/Goal.h>

// TimeSynchronizer includes
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "utils.hpp"

#include "faster.hpp"
#include "faster_types.hpp"

//####Class FasterRos
class FasterRos
{
public:
  FasterRos(ros::NodeHandle nh);

private:
  std::unique_ptr<Faster> faster_ptr_;

  // class methods
  void terminalGoalCB(const geometry_msgs::PoseStamped& msg);
  void stateCB(const snapstack_msgs::State& msg);
  void replanCB(const ros::TimerEvent& e);
  void mapCB(const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_msg,
             const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_msg2);  // Callback for the occupancy pcloud
  void unkCB(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg);     // Callback for the unkown pcloud
  void pclCB(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg);
  //void frontierCB(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg);

  //void updateInitialCond(int i);
  void publishJPSPath(vec_Vecf<3>& path);

  Eigen::Vector3d projectClickedGoal(Eigen::Vector3d& P1);
  void createMoreVertexes(vec_Vecf<3>& path, double d);

  void publishPoly(const vec_E<Polyhedron<3>>& poly,const vec_E<Ellipsoid<3>>& Ellips);

  std::string world_name_ = "world";

  ros::NodeHandle nh_;
  ros::Publisher poly_whole_pub_;
  ros::Publisher ellip_whole_pub_;
  ros::Publisher pub_global_plan;

  // ros::Publisher cvx_decomp_poly_uo_pub_;
  ros::Subscriber sub_goal_;
  ros::Subscriber sub_state_;

  // Eigen::Vector3d accel_vicon_;
  ros::Timer replanCBTimer_;

  parameters par_;  // where all the parameters are
  // snapstack_msgs::Cvx log_;  // to log all the data
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener* tfListener;
  std::string name_drone_;

  message_filters::Subscriber<sensor_msgs::PointCloud2> occup_grid_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> unknown_grid_sub_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>
      MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  //int actual_trajID_ = 0;
  // faster_msgs::Mode mode_;
};
