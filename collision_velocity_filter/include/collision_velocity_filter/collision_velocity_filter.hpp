/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef COB_COLLISION_VELOCITY_FILTER_H
#define COB_COLLISION_VELOCITY_FILTER_H


//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <rclcpp/rclcpp.hpp>

// ROS message includes
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

///
/// @class CollisionVelocityFilter
/// @brief checks for obstacles in driving direction and stops the robot
///
///
class CollisionVelocityFilter : public rclcpp::Node
{
public:

  ///
  /// @brief  Constructor
  ///
  CollisionVelocityFilter();

  ///
  /// @brief  reads twist command from teleop device (joystick, teleop_keyboard, ...) and calls functions
  ///         for collision check (obstacleHandler) and driving of the robot (performControllerStep)
  /// @param  twist - velocity command sent as twist message (twist.linear.x/y/z, twist.angular.x/y/z)
  ///
  void joystickVelocityCB(const std::shared_ptr<const geometry_msgs::msg::Twist> &twist);

  ///
  /// @brief  reads obstacles from costmap
  /// @param  obstacles - 2D occupancy grid in rolling window mode!
  ///
  void readObstacles();


  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr topic_pub_command_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr topic_pub_relevant_obstacles_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joystick_velocity_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;

private:

  ///
  /// @brief  checks distance to obstacles in driving direction and slows down/stops
  ///         robot and publishes command velocity to robot
  ///
  void performControllerStep();

  ///
  /// @brief  checks for obstacles in driving direction of the robot (rotation included)
  ///         and publishes relevant obstacles
  ///
  void obstacleHandler();

  ///
  /// @brief  returns the sign of x
  ///
  double sign(double x);

  ///
  /// @brief  computes distance between two points
  /// @param  a,b - Points
  /// @return distance
  ///
  double getDistance2d(geometry_msgs::msg::Point a, geometry_msgs::msg::Point b);

  ///
  /// @brief  checks if obstacle lies already within footprint -> this is ignored due to sensor readings of the hull etc
  /// @param  x_obstacle - x coordinate of obstacle in occupancy grid local costmap
  /// @param  y_obstacle - y coordinate of obstacle in occupancy grid local costmap
  /// @return true if obstacle outside of footprint
  ///
  bool obstacleValid(double x_obstacle, double y_obstacle);

  ///
  /// @brief  stops movement of the robot
  ///
  void stopMovement();

  //obstacle_treshold
  int costmap_obstacle_treshold_;

  //frames
  std::string global_frame_, robot_frame_;

  //velocity
  geometry_msgs::msg::Vector3 robot_twist_linear_, robot_twist_angular_;
  double v_max_, vtheta_max_;
  double ax_max_, ay_max_, atheta_max_;

  //obstacle avoidance
  std::shared_ptr<nav_msgs::msg::OccupancyGrid> local_costmap_;
  nav_msgs::msg::OccupancyGrid relevant_obstacles_;
  bool costmap_received_;
  std::shared_ptr<geometry_msgs::msg::PolygonStamped> robot_footprint_;
  double footprint_left_, footprint_right_, footprint_front_, footprint_rear_;
  double influence_radius_, stop_threshold_, obstacle_damping_dist_, use_circumscribed_threshold_;
  double closest_obstacle_dist_, closest_obstacle_angle_;

  // variables for slow down behavior
  rclcpp::Time last_time_;
  double kp_, kv_;
  double vx_last_, vy_last_, vtheta_last_;
  double virt_mass_;

  // BUT velocity limited marker
  //cob_collision_velocity_filter::VelocityLimitedMarker velocity_limited_marker_;

};
//CollisionVelocityFilter

#endif
