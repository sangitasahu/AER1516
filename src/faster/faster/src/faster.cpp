/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "faster.hpp"

#include <pcl/kdtree/kdtree.h>
#include <Eigen/StdVector>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <stdlib.h>

using namespace JPS;
using namespace termcolor;

// Uncomment the type of timer you want:
// typedef ROSTimer MyTimer;
// typedef ROSWallTimer MyTimer;
typedef Timer MyTimer;

Faster::Faster(parameters par) : par_(par)
{
  drone_status_ == DroneStatus::YAWING;
  // mtx_G.lock();
  G_.pos << 0, 0, 0;
  // mtx_G.unlock();
  G_term_.pos << 0, 0, 0;

  mtx_initial_cond.lock();
  stateA_.setZero();
  mtx_initial_cond.unlock();

  // Setup of jps_manager
  std::cout << "par_.wdx / par_.res =" << par_.wdx / par_.res << std::endl;
  jps_manager_.setNumCells((int)par_.wdx / par_.res, (int)par_.wdy / par_.res, (int)par_.wdz / par_.res);
  jps_manager_.setFactorJPS(par_.factor_jps);
  jps_manager_.setResolution(par_.res);
  jps_manager_.setInflationJPS(par_.inflation_jps);
  jps_manager_.setZGroundAndZMax(par_.z_ground, par_.z_max);
  // jps_manager_.setVisual(par_.visual);
  jps_manager_.setDroneRadius(par_.drone_radius);

  double max_values[3] = { par_.v_max, par_.a_max, par_.j_max };
  pclptr_unk_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pclptr_map_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  changeDroneStatus(DroneStatus::GOAL_REACHED);
  resetInitialization();
}

void Faster::createMoreVertexes(vec_Vecf<3>& path, double d)
{
  for (int j = 0; j < path.size() - 1; j++)
  {
    double dist = (path[j + 1] - path[j]).norm();
    int vertexes_to_add = floor(dist / d);
    Eigen::Vector3d v = (path[j + 1] - path[j]).normalized();
    // std::cout << "Vertexes to add=" << vertexes_to_add << std::endl;
    if (dist > d)
    {
      for (int i = 0; i < vertexes_to_add; i++)
      {
        path.insert(path.begin() + j + 1, path[j] + v * d);
        j = j + 1;
      }
    }
  }
}

void Faster::updateMap(pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_map, pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_unk)
{
  mtx_map.lock();
  mtx_unk.lock();

  // TODO
  pclptr_map_ = pclptr_map;
  pclptr_unk_ = pclptr_unk;

  jps_manager_.updateJPSMap(pclptr_map_, state_.pos);  // Update even where there are no points

  if (pclptr_map_->width != 0 && pclptr_map_->height != 0)  // Point Cloud is not empty
  {
    kdtree_map_.setInputCloud(pclptr_map_);
    kdtree_map_initialized_ = 1;
    jps_manager_.vec_o_ = pclptr_to_vec(pclptr_map_);
  }
  else
  {
    std::cout << "Occupancy Grid received is empty, maybe map is too small?" << std::endl;
  }

  if (pclptr_unk_->points.size() == 0)
  {
    std::cout << "Unkown cloud has 0 points" << std::endl;
    return;
  }
  else
  {
    kdtree_unk_.setInputCloud(pclptr_unk_);
    kdtree_unk_initialized_ = 1;
    jps_manager_.vec_uo_ = pclptr_to_vec(pclptr_unk_);  // insert unknown space
    jps_manager_.vec_uo_.insert(jps_manager_.vec_uo_.end(), jps_manager_.vec_o_.begin(),
                                jps_manager_.vec_o_.end());  // append known space
  }

  mtx_map.unlock();
  mtx_unk.unlock();
}

void Faster::setTerminalGoal(state& term_goal)
{
  mtx_G_term.lock();
  mtx_G.lock();
  mtx_state.lock();
  mtx_planner_status_.lock();

  G_term_.pos = term_goal.pos;
  Eigen::Vector3d temp = state_.pos;
  G_.pos = projectPointToBox(temp, G_term_.pos, par_.wdx, par_.wdy, par_.wdz);
  if (drone_status_ == DroneStatus::GOAL_REACHED)
  {
    changeDroneStatus(DroneStatus::YAWING);  // not done when drone_status==traveling
  }
  terminal_goal_initialized_ = true;

  mtx_state.unlock();
  mtx_G.unlock();
  mtx_G_term.unlock();
  mtx_planner_status_.unlock();
}

void Faster::getG(state& G)
{
  G = G_;
}

void Faster::getState(state& data)
{
  mtx_state.lock();
  data = state_;
  mtx_state.unlock();
}

void Faster::updateState(state data)
{
  state_ = data;
  state tmp;
  tmp.pos = data.pos;
  tmp.yaw = data.yaw;
  plan_.push_back(tmp);
  state_initialized_ = true;
}

bool Faster::initializedAllExceptPlanner()
{
  return true;
}

bool Faster::initialized()
{
  return true;
}

void Faster::replan(vec_Vecf<3>& JPS_whole_out,vec_E<Polyhedron<3>>& poly_whole_out)
{
  MyTimer replanCB_t(true);
  if (initializedAllExceptPlanner() == false)
  {
    return;
  }
  mtx_state.lock();
  mtx_G.lock();
  mtx_G_term.lock();

  state state_local = state_;
  state G;
  G.pos = projectPointToBox(state_local.pos, G_term_.pos, par_.wdx, par_.wdy, par_.wdz);
  state G_term = G_term_;  // Local copy of the terminal terminal goal

  mtx_G.unlock();
  mtx_G_term.unlock();
  mtx_state.unlock();

  // Check if we have reached the goal
  double dist_to_goal = (G_term.pos - state_local.pos).norm();
  if (dist_to_goal < par_.goal_radius)
  {
    changeDroneStatus(DroneStatus::GOAL_REACHED);
  }

  // Don't plan if drone is not traveling
  if (drone_status_ == DroneStatus::GOAL_REACHED )// || (drone_status_ == DroneStatus::YAWING))
  {
    std::cout << "GOAL_REACHED" << std::endl;
    return;
  }

  //std::cout << bold << on_red << "************IN REPLAN CB*********" << reset << std::endl;

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Select state A /////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  state A;
  int k_safe, k_end_whole;

  // If k_end_whole=0, then A = plan_.back() = plan_[plan_.size() - 1]
  k_end_whole = std::max((int)plan_.size() - deltaT_, 0);
  A = plan_[plan_.size() - 1 - k_end_whole];

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Solve JPS //////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  bool solvedjps = false;
  MyTimer timer_jps(true);

  //std::cout << "JPS starting from: (" << state_local.pos[0] << "),(" << state_local.pos[1] << "),(" << state_local.pos[2] << ")" << std::endl;

  // vec_Vecf<3> JPSk = jps_manager_.solveJPS3D(A.pos, G.pos, &solvedjps, 1);
  vec_Vecf<3> JPSk = jps_manager_.solveJPS3D(state_local.pos, G.pos, &solvedjps, 1);

  if (solvedjps == false)
  {
    std::cout << bold << red << "JPS didn't find a solution" << std::endl;
    return;
  }
  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Find JPS_in ////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  double ra = std::min((dist_to_goal - 0.001), par_.Ra);  // radius of the sphere S
  bool noPointsOutsideS;
  int li1;  // last index inside the sphere of JPSk
  state E;
  E.pos = getFirstIntersectionWithSphere(JPSk, ra, JPSk[0], &li1, &noPointsOutsideS);
  vec_Vecf<3> JPS_in(JPSk.begin(), JPSk.begin() + li1 + 1);
  if (noPointsOutsideS == false)
  {
    JPS_in.push_back(E.pos);
  }
  // createMoreVertexes in case dist between vertexes is too big
  createMoreVertexes(JPS_in, par_.dist_max_vertexes);
  vec_Vecf<3> JPS_whole = JPS_in;
  deleteVertexes(JPS_whole, par_.max_poly_whole);
  E.pos = JPS_whole[JPS_whole.size() - 1];
    // Convex Decomp around JPS_whole
  MyTimer cvx_ellip_decomp_t(true);
  jps_manager_.cvxEllipsoidDecomp(JPS_whole, OCCUPIED_SPACE, l_constraints_whole_, poly_whole_out);
  // Check if G is inside poly_whole
  bool isGinside_whole = l_constraints_whole_[l_constraints_whole_.size() - 1].inside(G.pos);
  E.pos = (isGinside_whole == true) ? G.pos : E.pos;
  JPS_whole_out = JPS_whole;
  return;
}

void Faster::resetInitialization(){
  planner_initialized_ = false;
  state_initialized_ = false;
  kdtree_map_initialized_ = false;
  kdtree_unk_initialized_ = false;
  terminal_goal_initialized_ = false;
}

// Debugging functions
void Faster::changeDroneStatus(int new_status)
{
  if (new_status == drone_status_)
  {
    return;
  }

  //std::cout << "Changing DroneStatus from ";
  switch (drone_status_)
  {
    case DroneStatus::YAWING:
      //std::cout << bold << "status_=YAWING" << reset;
      break;
    case DroneStatus::TRAVELING:
      //std::cout << bold << "status_=TRAVELING" << reset;
      break;
    case DroneStatus::GOAL_SEEN:
      //std::cout << bold << "status_=GOAL_SEEN" << reset;
      break;
    case DroneStatus::GOAL_REACHED:
      //std::cout << bold << "status_=GOAL_REACHED" << reset;
      break;
  }
  //std::cout << " to ";

  switch (new_status)
  {
    case DroneStatus::YAWING:
      //std::cout << bold << "status_=YAWING" << reset;
      break;
    case DroneStatus::TRAVELING:
      //std::cout << bold << "status_=TRAVELING" << reset;
      break;
    case DroneStatus::GOAL_SEEN:
      //std::cout << bold << "status_=GOAL_SEEN" << reset;
      break;
    case DroneStatus::GOAL_REACHED:
      //std::cout << bold << "status_=GOAL_REACHED" << reset;
      break;
  }

  std::cout << std::endl;

  drone_status_ = new_status;
}
