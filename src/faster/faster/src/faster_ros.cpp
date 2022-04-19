/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "faster_ros.hpp"
#include <sensor_msgs/point_cloud_conversion.h>

// This object is created in the faster_ros_node
FasterRos::FasterRos(ros::NodeHandle nh) : nh_(nh)
{
  safeGetParam(nh_, "use_ff", par_.use_ff);
  safeGetParam(nh_, "visual", par_.visual);

  safeGetParam(nh_, "dc", par_.dc);
  safeGetParam(nh_, "goal_radius", par_.goal_radius);
  safeGetParam(nh_, "drone_radius", par_.drone_radius);
  safeGetParam(nh_, "force_goal_height", par_.force_goal_height);
  safeGetParam(nh_, "goal_height", par_.goal_height);

  safeGetParam(nh_, "N_safe", par_.N_safe);
  safeGetParam(nh_, "N_whole", par_.N_whole);

  safeGetParam(nh_, "Ra", par_.Ra);
  safeGetParam(nh_, "w_max", par_.w_max);
  safeGetParam(nh_, "alpha_filter_dyaw", par_.alpha_filter_dyaw);

  safeGetParam(nh_, "z_ground", par_.z_ground);
  safeGetParam(nh_, "z_max", par_.z_max);
  safeGetParam(nh_, "inflation_jps", par_.inflation_jps);
  safeGetParam(nh_, "factor_jps", par_.factor_jps);

  safeGetParam(nh_, "v_max", par_.v_max);
  safeGetParam(nh_, "a_max", par_.a_max);
  safeGetParam(nh_, "j_max", par_.j_max);

  safeGetParam(nh_, "gamma_whole", par_.gamma_whole);
  safeGetParam(nh_, "gammap_whole", par_.gammap_whole);
  safeGetParam(nh_, "increment_whole", par_.increment_whole);
  safeGetParam(nh_, "gamma_safe", par_.gamma_safe);
  safeGetParam(nh_, "gammap_safe", par_.gammap_safe);
  safeGetParam(nh_, "increment_safe", par_.increment_safe);

  safeGetParam(nh_, "delta_a", par_.delta_a);
  safeGetParam(nh_, "delta_H", par_.delta_H);

  safeGetParam(nh_, "max_poly_whole", par_.max_poly_whole);
  safeGetParam(nh_, "max_poly_safe", par_.max_poly_safe);
  safeGetParam(nh_, "dist_max_vertexes", par_.dist_max_vertexes);

  safeGetParam(nh_, "gurobi_threads", par_.gurobi_threads);
  safeGetParam(nh_, "gurobi_verbose", par_.gurobi_verbose);

  safeGetParam(nh_, "use_faster", par_.use_faster);

  safeGetParam(nh_, "is_ground_robot", par_.is_ground_robot);

  // And now obtain the parameters from the mapper
  std::vector<double> world_dimensions;
  safeGetParam(nh_, "world_dimensions", world_dimensions);
  safeGetParam(nh_, "resolution", par_.res);
  
  safeGetParam(nh_, "setup/jps_3d_on_goal_loc", par_.pub_goal_set_name);

  par_.wdx = world_dimensions[0];
  par_.wdy = world_dimensions[1];
  par_.wdz = world_dimensions[2];

  std::cout << bold << green << "world_dimensions=" << world_dimensions << reset << std::endl;
  std::cout << bold << green << "resolution=" << par_.res << reset << std::endl;

  std::cout << "Parameters obtained" << std::endl;

  if (par_.N_safe <= par_.max_poly_safe + 2)
  {
    std::cout << bold << red << "Needed: N_safe>=max_poly+ 2 at least" << reset
              << std::endl;  // To decrease the probability of not finding a solution
    abort();
  }
  if (par_.N_whole <= par_.max_poly_whole + 2)
  {
    std::cout << bold << red << "Needed: N_whole>=max_poly + 2 at least" << reset
              << std::endl;  // To decrease the probability of not finding a solution
    abort();
  }

  if (par_.factor_jps * par_.res / 2.0 > par_.inflation_jps)
  {
    std::cout << bold << red << "Needed: par_.factor_jps * par_.res / 2 <= par_.inflation_jps" << reset
              << std::endl;  // If not JPS will find a solution between the voxels.
    abort();
  }

  /*  if (par_.Ra_max > (par_.wdx / 2.0) || (par_.Ra_max > par_.wdy / 2.0))
    {
      std::cout << bold << red << "Needed: par_.Ra_max > par_.wdx/2.0|| par_.Ra_max > par_.wdy/2.0" << reset
                << std::endl;  // To decrease the probability of not finding a solution
      abort();f
    }*/

  /*  if (par_.drone_radius <= 2 * par_.res)
    {
      std::cout << bold << red << "Needed: par_.drone_radius > 2*par_.res" << reset
                << std::endl;  // If not the convex decomposition finds polytopes between the voxels of the obstacles
      abort();
    }*/

  /*  if (par_.inflation_jps <= par_.res/2.0 + par_.drone_radius)
    {
      std::cout << bold << red << "Needed: par_.inflation_jps > par_.res/2.0 + par_.drone_radius" << reset
                << std::endl; //JPS should be run with at least drone_radius + half of the size of a voxel
      abort();
    }
  */

  // Initialize FASTER
  faster_ptr_ = std::unique_ptr<Faster>(new Faster(par_));
  ROS_INFO("Planner initialized");

  // Publishers
  poly_whole_pub_ = nh.advertise<decomp_ros_msgs::PolyhedronArray>("poly_whole", 1, true);
  ellip_whole_pub_ = nh.advertise<decomp_ros_msgs::EllipsoidArray>("Ellip_whole", 1, true);
  pub_global_plan = nh_.advertise<nav_msgs::Path>("global_plan", 1);



  // Subscribers
  occup_grid_sub_.subscribe(nh_, "/occup_grid", 1);
  unknown_grid_sub_.subscribe(nh_, "/unknown_grid", 1);
  sync_.reset(new Sync(MySyncPolicy(1), occup_grid_sub_, unknown_grid_sub_));
  sync_->registerCallback(boost::bind(&FasterRos::mapCB, this, _1, _2));
  sub_goal_ = nh_.subscribe("/move_base_simple/goal", 1, &FasterRos::terminalGoalCB, this);
    if (par_.pub_goal_set_name == true)
  {
    sub_goal_ = nh_.subscribe("/goal_loc", 1, &FasterRos::terminalGoalCB, this);
  }

  sub_state_ = nh_.subscribe("state", 1, &FasterRos::stateCB, this);
  // Timers
  replanCBTimer_ = nh_.createTimer(ros::Duration(par_.dc), &FasterRos::replanCB, this);
}

void FasterRos::replanCB(const ros::TimerEvent& e){
  if (ros::ok())
  {
    vec_Vecf<3> JPS_whole;
    vec_E<Polyhedron<3>> poly_whole;
    vec_E<Ellipsoid<3>> ellips_whole;

    faster_ptr_->replan(JPS_whole,poly_whole,ellips_whole);
    publishJPSPath(JPS_whole); //Vandan Added this
    publishPoly(poly_whole,ellips_whole);
    
  }
}
void FasterRos::publishPoly(const vec_E<Polyhedron<3>>& poly,const vec_E<Ellipsoid<3>>& Ellips){
  // std::cout << "Going to publish= " << (poly[0].hyperplanes())[0].n_ << std::endl;
  decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(poly);
  poly_msg.header.frame_id = world_name_;
  poly_whole_pub_.publish(poly_msg);
  decomp_ros_msgs::EllipsoidArray ellip_msg = DecompROS::ellipsoid_array_to_ros(Ellips);
  ellip_msg.header.frame_id = world_name_;
  ellip_whole_pub_.publish(ellip_msg);
  
}
void FasterRos::stateCB(const snapstack_msgs::State& msg){
  state state_tmp;
  state_tmp.setPos(msg.pos.x, msg.pos.y, msg.pos.z);
  state_tmp.setVel(msg.vel.x, msg.vel.y, msg.vel.z);
  state_tmp.setAccel(0.0, 0.0, 0.0);
  double roll, pitch, yaw;
  quaternion2Euler(msg.quat, roll, pitch, yaw);
  state_tmp.setYaw(yaw);
  faster_ptr_->updateState(state_tmp);
}
void FasterRos::publishJPSPath(vec_Vecf<3>& path){
  nav_msgs::Path globalplan;
  geometry_msgs::PoseStamped pose;
  //pose.header.stamp = ros::Time::now();
  //pose.header.frame_id = "vicon";
  globalplan.header.stamp =  ros::Time::now();
  globalplan.header.frame_id = "vicon";
  for (const auto& it : path)
  {
    geometry_msgs::Point p = eigen2point(it);
    pose.pose.position = p;
    globalplan.poses.push_back(pose);
  }
  pub_global_plan.publish(globalplan);
}
// Occupied CB
void FasterRos::mapCB(const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_map_ros,
                      const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_unk_ros){
  // Occupied Space Point Cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_map(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pcl2ptr_map_ros, *pclptr_map);
  // Unknown Space Point Cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_unk(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pcl2ptr_unk_ros, *pclptr_unk);

  faster_ptr_->updateMap(pclptr_map, pclptr_unk);
}
void FasterRos::terminalGoalCB(const geometry_msgs::PoseStamped& msg){
  state G_term;

  double height;
  if (par_.is_ground_robot)
  {
    height = 0.2;
  }
  else if (par_.force_goal_height)
  {
    height = par_.goal_height;
  }
  else
  {
    height = msg.pose.position.z;
  }

  // const double height = (par_.is_ground_robot) ? 0.2 : goal_height_;
  G_term.setPos(msg.pose.position.x, msg.pose.position.y, height);
  faster_ptr_->setTerminalGoal(G_term);

  state G;  // projected goal
  faster_ptr_->getG(G);
}
