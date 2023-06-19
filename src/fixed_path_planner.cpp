/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Derived from Straight line planner tutorial by Shivang Patel
 *
 * Original tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/array_parser.hpp"

#include "nav2_fixed_path_planner/fixed_path_planner.hpp"


namespace {
// When selecting the next path point, only consider points within
// this max angle off of the current pose  
const double MAX_OFF_HEADING_ANGLE = 45.0/180.0*M_PI;
}

namespace nav2_fixed_path_planner
{
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

void FixedPath::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);

  std::string waypoints_param;
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".waypoints", rclcpp::ParameterValue(std::string("")));
  node_->get_parameter(name_ + ".waypoints", waypoints_param);

  parseWaypoints(waypoints_param, waypoints_);
}

void FixedPath::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void FixedPath::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
  // Add callback for dynamic parameters
  _dyn_params_handler = node_->add_on_set_parameters_callback(
    std::bind(&FixedPath::dynamicParametersCallback, this, _1));
}

void FixedPath::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

double FixedPath::to_degrees(double rads) const
{
  return rads/M_PI*180.0;
}

int FixedPath::FindClosestOnPath(
  const std::vector<Waypoint> & waypoints,
  const geometry_msgs::msg::PoseStamped & position,
  bool closest_in_front, double ref_angle)
{
  struct PointAttribs {
    PointAttribs(int idx, double angle, double dist):
      idx(idx),
      angle(angle),
      dist(dist)
    {}
    int idx;
    double angle;
    double dist;
  };

  // Calc angle off from current pose and distance to each path point
  std::vector<PointAttribs> point_attribs;
  for (int i = 0; i < waypoints.size(); i++) {
    point_attribs.emplace_back(PointAttribs(i, atan2(waypoints[i].y - position.pose.position.y, waypoints[i].x - position.pose.position.x),
                                         std::hypot(waypoints[i].y - position.pose.position.y, waypoints[i].x - position.pose.position.x)));
    RCLCPP_INFO(node_->get_logger(), "Point %d, (%f, %f), angle: %f, dist: %f", i, waypoints[i].x, waypoints[i].y,
      to_degrees(point_attribs.back().angle), point_attribs.back().dist);
  }

  for (auto &a: point_attribs) {
    a.angle -= ref_angle;
    RCLCPP_DEBUG(node_->get_logger(), "Diff angle-yaw: %f", to_degrees(a.angle));
  }

  sort(point_attribs.begin(), point_attribs.end(), 
    [](const auto &a, const auto &b)
    { 
      return abs(a.dist) < abs(b.dist); 
    });

#ifdef EXTRA_DEBUG
  RCLCPP_INFO(node_->get_logger(), "Sorted waypoints by dist:");
  for (const auto &a: point_attribs) {
    RCLCPP_INFO(node_->get_logger(), "idx: %d, angle: %f, dist: %f", a.idx, to_degrees(a.angle), a.dist);
  }
#endif  

  
  auto idx = -1;

  for (const auto &a: point_attribs) {
    if (!closest_in_front || abs(a.angle) < MAX_OFF_HEADING_ANGLE) {
      idx = a.idx;
      RCLCPP_INFO(node_->get_logger(), "Closest point is idx: %d, angle: %f, dist: %f", a.idx, to_degrees(a.angle), a.dist);
      break;
    }
  }

  // Use the closest if no point is withing the heading tolerance
  if (idx == -1) {
    idx = point_attribs.front().idx;
  }
  return idx;
}


nav_msgs::msg::Path FixedPath::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  double yaw = tf2::getYaw(start.pose.orientation);
  RCLCPP_INFO(node_->get_logger(), "start pose angle: %f", to_degrees(yaw));

  // Make a copy of params since they can change via dynamic reconfig while in use
  double interpolation_resolution = 0.0;
  std::vector<Waypoint> waypoints;
  {
    std::mutex _mutex;
    interpolation_resolution = interpolation_resolution_;
    waypoints = waypoints_;
  }

  // Algorithm:
  //  1. Calc distance and angle offset from current heading for each path waypoint
  //  2. Sort list by distance
  //  3. Pick closest waypoint in front of current position
  //  4. Create path starting with the closest waypoint
  //  5. Stop at the waypoint closest to the specified goal

  auto idx = FindClosestOnPath(waypoints, start, true, yaw);
  auto idx_last = FindClosestOnPath(waypoints, goal, false, 0.0);


  double prev_x = start.pose.position.x;
  double prev_y = start.pose.position.y;

  for (int p = 0; p < waypoints.size(); p++) {
    // Calculate the number of interpolation loops between the next pairs of path points
    int total_number_of_loop = std::hypot(waypoints[idx].x - prev_x, waypoints[idx].y - prev_y) /
      interpolation_resolution;
    
    double x_increment = (waypoints[idx].x - prev_x) / total_number_of_loop;
    double y_increment = (waypoints[idx].y - prev_y) / total_number_of_loop;

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;

    // Generate the points between the current pair
    for (int i = 0; i < total_number_of_loop; ++i) {
      pose.pose.position.x =  prev_x + x_increment * i;
      pose.pose.position.y =  prev_y + y_increment * i;
      global_path.poses.push_back(pose);
    }

    prev_x = waypoints[idx].x;
    prev_y = waypoints[idx].y;

    if (idx == idx_last) {
      break;
    }

    if (++idx >= waypoints.size()) {
      idx = 0;
    }
  }

  return global_path;
}

rcl_interfaces::msg::SetParametersResult
FixedPath::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(_mutex);

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == name_ + ".interpolation_resolution") {
        interpolation_resolution_ = static_cast<float>(parameter.as_double());
      }

    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == name_ + ".waypoints") {
        parseWaypoints(parameter.as_string(), waypoints_);
      }
    }
  }

  result.successful = true;
  return result;
}

bool FixedPath::parseWaypoints(
  const std::string & waypoints_string,
  std::vector<Waypoint> &waypoints) const
{
  std::string error;
  std::vector<std::vector<float>> vvf = nav2_costmap_2d::parseVVF(waypoints_string, error);

  if (error != "") {
    RCLCPP_ERROR(
      node_->get_logger(), "Error parsing waypoints parameter: '%s'", error.c_str());
    return false;
  }

  waypoints.clear();

  // convert vvf into points.
  if (vvf.size() < 1) {
    RCLCPP_ERROR(
      node_->get_logger(), 
      "You must specify at least one waypoint.");
    return false;
  }

  waypoints.reserve(vvf.size());
  for (unsigned int i = 0; i < vvf.size(); i++) {
    if (vvf[i].size() == 2) {
      waypoints.emplace_back(vvf[i][0], vvf[i][1]);
    } else {
      RCLCPP_ERROR(
        node_->get_logger(), 
        "Waypoints must be pairs of numbers. Found a point with %d numbers.",
        static_cast<int>(vvf[i].size()));
      return false;
    }
  }

  for (int i = 0; i < waypoints_.size(); i++) {
    RCLCPP_INFO(
      node_->get_logger(), "Waypoint: %d, (%f, %f)",
      i, waypoints_[i].x, waypoints_[i].y);
  }

  return true;
}


}  // namespace nav2_fixed_path_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_fixed_path_planner::FixedPath, nav2_core::GlobalPlanner)
