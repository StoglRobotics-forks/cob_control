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


#include "collision_velocity_filter/collision_velocity_filter.hpp"

// Constructor
CollisionVelocityFilter::CollisionVelocityFilter()
  : Node("collision_velocity_filter")
{
    /// PARAMETERS:
  costmap_obstacle_treshold_ = 250;
  global_frame_ = "map";
  robot_frame_ = "base_link";
  
  influence_radius_ = 1.5;
  closest_obstacle_dist_ = influence_radius_;
  closest_obstacle_angle_ = 0.0;
  
  stop_threshold_ = 0.1;
  obstacle_damping_dist_ = 5.0;
  if (obstacle_damping_dist_ <= stop_threshold_)
  {
    obstacle_damping_dist_ = stop_threshold_ + 0.01; // set to stop_threshold_+0.01 to avoid divide by zero error
    RCLCPP_WARN(this->get_logger(), 
      "obstacle_damping_dist <= stop_threshold -> robot will stop without deceleration!");
  }

  use_circumscribed_threshold_ = 0.2;
  v_max_ = 0.6;
  vtheta_max_ = 0.8;
  kv_ = 1.0;
  kp_ = 2.0;
  virt_mass_ = 0.8;

  ax_max_ = 0.5;
  ay_max_ = 0.5;
  atheta_max_ = 0.7;
  vx_last_ = 0.0;
  vy_last_ = 0.0;
  vtheta_last_ = 0.0;
  last_time_ = this->now();

  footprint_right_ = 0.0;
  footprint_left_ = 0.0;
  footprint_front_ = 0.0;
  footprint_rear_ = 0.0;

  /// SUBSCRIBERS:
  joystick_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "~/command_in", rclcpp::SystemDefaultsQoS(),
    std::bind(&CollisionVelocityFilter::joystickVelocityCB, this, std::placeholders::_1));

  auto localCostmapCB =
    [this](const std::shared_ptr<nav_msgs::msg::OccupancyGrid> msg)
  { local_costmap_ = msg; };
  local_costmap_sub_ =
    this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_costmap/costmap", rclcpp::SystemDefaultsQoS(), localCostmapCB);

  auto footprintCB =
    [this](const std::shared_ptr<geometry_msgs::msg::PolygonStamped> msg)
  { 
    if (msg->polygon.points.size() > 4)
      RCLCPP_WARN(this->get_logger(),
        "You are trying to set more than 4 points as robot_footprint, collision_velocity_filter can deal only with rectangular footprints so far!");
    else{
      robot_footprint_ = msg;
        for (unsigned int i = 0; i < msg->polygon.points.size(); i++)
      {
        if (robot_footprint_->polygon.points[i].x > footprint_front_)
          footprint_front_ = robot_footprint_->polygon.points[i].x;
        if (robot_footprint_->polygon.points[i].x < footprint_rear_)
          footprint_rear_ = robot_footprint_->polygon.points[i].x;
        if (robot_footprint_->polygon.points[i].y > footprint_left_)
          footprint_left_ = robot_footprint_->polygon.points[i].y;
        if (robot_footprint_->polygon.points[i].y < footprint_right_)
          footprint_right_ = robot_footprint_->polygon.points[i].y;
      }
    }
  };
  footprint_sub_ =
    this->create_subscription<geometry_msgs::msg::PolygonStamped>(
      "/local_costmap/published_footprint", rclcpp::SystemDefaultsQoS(), footprintCB);

  /// PUBLISHERS:
  topic_pub_command_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("~/command", 1);
  topic_pub_relevant_obstacles_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/relevant_obstacles", 1);

  //TODO: set info
  RCLCPP_INFO(this->get_logger(),
    "collision_velocity_filter initialized.");
}

// joystick_velocityCB reads twist command from joystick
void CollisionVelocityFilter::joystickVelocityCB(const std::shared_ptr<const geometry_msgs::msg::TwistStamped> &twist)
{
  //std::cout << "received command" << std::endl;
  RCLCPP_INFO(this->get_logger(),
    "Received joystick command.");

  robot_twist_linear_ = twist->linear;
  robot_twist_angular_ = twist->angular;

  std::cout<<"i hear:"<<robot_twist_linear_.x<<std::endl;

  // check for relevant obstacles
  obstacleHandler();
  /*
  // stop if we are about to run in an obstacle
  performControllerStep();
  */
}

/*
// sets corrected velocity of joystick command
void CollisionVelocityFilter::performControllerStep()
{

  double dt;
  double vx_max, vy_max;
  geometry_msgs::msg::TwistStamped cmd_vel, cmd_vel_in;

  cmd_vel_in.linear = robot_twist_linear_;
  cmd_vel_in.angular = robot_twist_angular_;

  cmd_vel.linear = robot_twist_linear_;
  cmd_vel.angular = robot_twist_angular_;
  dt = ros::Time::now().toSec() - last_time_;
  last_time_ = ros::Time::now().toSec();

  double vel_angle = atan2(cmd_vel.linear.y, cmd_vel.linear.x);
  vx_max = v_max_ * fabs(cos(vel_angle));
  if (vx_max > fabs(cmd_vel.linear.x))
    vx_max = fabs(cmd_vel.linear.x);
  vy_max = v_max_ * fabs(sin(vel_angle));
  if (vy_max > fabs(cmd_vel.linear.y))
    vy_max = fabs(cmd_vel.linear.y);

  //Slow down in any way while approximating an obstacle:
  if (closest_obstacle_dist_ < influence_radius_)
  {
    double F_x, F_y;
    double vx_d, vy_d, vx_factor, vy_factor;
    double kv_obst = kv_, vx_max_obst = vx_max, vy_max_obst = vy_max;

    //implementation for linear decrease of v_max:
    double obstacle_linear_slope_x = vx_max / (obstacle_damping_dist_ - stop_threshold_);
    vx_max_obst = (closest_obstacle_dist_ - stop_threshold_ + stop_threshold_ / 10.0f) * obstacle_linear_slope_x;
    if (vx_max_obst > vx_max)
      vx_max_obst = vx_max;
    else if (vx_max_obst < 0.0f)
      vx_max_obst = 0.0f;

    double obstacle_linear_slope_y = vy_max / (obstacle_damping_dist_ - stop_threshold_);
    vy_max_obst = (closest_obstacle_dist_ - stop_threshold_ + stop_threshold_ / 10.0f) * obstacle_linear_slope_y;
    if (vy_max_obst > vy_max)
      vy_max_obst = vy_max;
    else if (vy_max_obst < 0.0f)
      vy_max_obst = 0.0f;

    //Translational movement
    //calculation of v factor to limit maxspeed
    double closest_obstacle_dist_x = closest_obstacle_dist_ * cos(closest_obstacle_angle_);
    double closest_obstacle_dist_y = closest_obstacle_dist_ * sin(closest_obstacle_angle_);
    vx_d = kp_ / kv_obst * closest_obstacle_dist_x;
    vy_d = kp_ / kv_obst * closest_obstacle_dist_y;
    vx_factor = vx_max_obst / sqrt(vy_d * vy_d + vx_d * vx_d);
    vy_factor = vy_max_obst / sqrt(vy_d * vy_d + vx_d * vx_d);
    if (vx_factor > 1.0)
      vx_factor = 1.0;
    if (vy_factor > 1.0)
      vy_factor = 1.0;

    F_x = -kv_obst * vx_last_ + vx_factor * kp_ * closest_obstacle_dist_x;
    F_y = -kv_obst * vy_last_ + vy_factor * kp_ * closest_obstacle_dist_y;

    cmd_vel.linear.x = vx_last_ + F_x / virt_mass_ * dt;
    cmd_vel.linear.y = vy_last_ + F_y / virt_mass_ * dt;

  }

  // make sure, the computed and commanded velocities are not greater than the specified max velocities
  if (fabs(cmd_vel.linear.x) > vx_max)
    cmd_vel.linear.x = sign(cmd_vel.linear.x) * vx_max;
  if (fabs(cmd_vel.linear.y) > vy_max)
    cmd_vel.linear.y = sign(cmd_vel.linear.y) * vy_max;
  if (fabs(cmd_vel.angular.z) > vtheta_max_)
    cmd_vel.angular.z = sign(cmd_vel.angular.z) * vtheta_max_;

  // limit acceleration:
  // only acceleration (in terms of speeding up in any direction) is limited,
  // deceleration (in terms of slowing down) is handeled either by cob_teleop or the potential field
  // like slow-down behaviour above
  if (fabs(cmd_vel.linear.x) > fabs(vx_last_))
  {
    if ((cmd_vel.linear.x - vx_last_) / dt > ax_max_)
      cmd_vel.linear.x = vx_last_ + ax_max_ * dt;
    else if ((cmd_vel.linear.x - vx_last_) / dt < -ax_max_)
      cmd_vel.linear.x = vx_last_ - ax_max_ * dt;
  }
  if (fabs(cmd_vel.linear.y) > fabs(vy_last_))
  {
    if ((cmd_vel.linear.y - vy_last_) / dt > ay_max_)
      cmd_vel.linear.y = vy_last_ + ay_max_ * dt;
    else if ((cmd_vel.linear.y - vy_last_) / dt < -ay_max_)
      cmd_vel.linear.y = vy_last_ - ay_max_ * dt;
  }
  if (fabs(cmd_vel.angular.z) > fabs(vtheta_last_))
  {
    if ((cmd_vel.angular.z - vtheta_last_) / dt > atheta_max_)
      cmd_vel.angular.z = vtheta_last_ + atheta_max_ * dt;
    else if ((cmd_vel.angular.z - vtheta_last_) / dt < -atheta_max_)
      cmd_vel.angular.z = vtheta_last_ - atheta_max_ * dt;
  }

  pthread_mutex_lock(&m_mutex);
  vx_last_ = cmd_vel.linear.x;
  vy_last_ = cmd_vel.linear.y;
  vtheta_last_ = cmd_vel.angular.z;
  pthread_mutex_unlock(&m_mutex);

  velocity_limited_marker_.publishMarkers(cmd_vel_in.linear.x, cmd_vel.linear.x, cmd_vel_in.linear.y, cmd_vel.linear.y,
                                          cmd_vel_in.angular.z, cmd_vel.angular.z);

  // if closest obstacle is within stop_threshold, then do not move
  if (closest_obstacle_dist_ < stop_threshold_)
  {
    stopMovement();
  }
  else
  {
    // publish adjusted velocity
    topic_pub_command_.publish(cmd_vel);
  }
  return;
}
*/


void CollisionVelocityFilter::obstacleHandler()
{
  closest_obstacle_dist_ = influence_radius_;

  double cur_distance_to_center, cur_distance_to_border;
  double obstacle_theta_robot, obstacle_delta_theta_robot, obstacle_dist_vel_dir;
  bool cur_obstacle_relevant;
  geometry_msgs::msg::Point cur_obstacle_robot;
  geometry_msgs::msg::Point zero_position;
  zero_position.x = 0.0f;
  zero_position.y = 0.0f;
  zero_position.z = 0.0f;
  bool use_circumscribed = true, use_tube = true;

  //Calculate corner angles in robot_frame:
  double corner_front_left, corner_rear_left, corner_rear_right, corner_front_right;
  corner_front_left = atan2(footprint_left_, footprint_front_);
  corner_rear_left = atan2(footprint_left_, footprint_rear_);
  corner_rear_right = atan2(footprint_right_, footprint_rear_);
  corner_front_right = atan2(footprint_right_, footprint_front_);

  //Decide, whether circumscribed or tube argument should be used for filtering:
  if (fabs(robot_twist_linear_.x) <= 0.005f && fabs(robot_twist_linear_.y) <= 0.005f)
  {
    use_tube = false;
    //disable tube filter at very slow velocities
  }
  if (!use_tube)
  {
    if (fabs(robot_twist_angular_.z) <= 0.01f)
    {
      use_circumscribed = false;
    } //when tube filter inactive, start circumscribed filter at very low rot-velocities
  }
  else
  {
    if (fabs(robot_twist_angular_.z) <= use_circumscribed_threshold_)
    {
      use_circumscribed = false;
    } //when tube filter running, disable circum-filter in a wider range of rot-velocities
  }

  //Calculation of tube in driving-dir considered for obstacle avoidence
  double velocity_angle = 0.0f, velocity_ortho_angle;
  double corner_angle, delta_corner_angle;
  double ortho_corner_dist;
  double tube_left_border = 0.0f, tube_right_border = 0.0f;
  double tube_left_origin = 0.0f, tube_right_origin = 0.0f;
  double corner_dist, circumscribed_radius = 0.0f;

  for (unsigned i = 0; i < robot_footprint_->polygon.points.size(); i++)
  {
    corner_dist = sqrt(robot_footprint_->polygon.points[i].x * robot_footprint_->polygon.points[i].x + robot_footprint_->polygon.points[i].y * robot_footprint_->polygon.points[i].y);
    if (corner_dist > circumscribed_radius)
      circumscribed_radius = corner_dist;
  }

  if (use_tube)
  {
    //use commanded vel-value for vel-vector direction.. ?
    velocity_angle = atan2(robot_twist_linear_.y, robot_twist_linear_.x);
    velocity_ortho_angle = velocity_angle + M_PI / 2.0f;

    for (unsigned i = 0; i < robot_footprint_->polygon.points.size(); i++)
    {
      corner_angle = atan2(robot_footprint_->polygon.points[i].y, robot_footprint_->polygon.points[i].x);
      delta_corner_angle = velocity_ortho_angle - corner_angle;
      corner_dist = sqrt(robot_footprint_->polygon.points[i].x * robot_footprint_->polygon.points[i].x + robot_footprint_->polygon.points[i].y * robot_footprint_->polygon.points[i].y);
      ortho_corner_dist = cos(delta_corner_angle) * corner_dist;

      if (ortho_corner_dist < tube_right_border)
      {
        tube_right_border = ortho_corner_dist;
        tube_right_origin = sin(delta_corner_angle) * corner_dist;
      }
      else if (ortho_corner_dist > tube_left_border)
      {
        tube_left_border = ortho_corner_dist;
        tube_left_origin = sin(delta_corner_angle) * corner_dist;
      }
    }
  }

  //find relevant obstacles
  relevant_obstacles_ = *local_costmap_;
  relevant_obstacles_.header.stamp = this->now();
  relevant_obstacles_.data.clear();
  for (unsigned int i = 0;
      i
          < local_costmap_->info.width
              * local_costmap_->info.height; i++)
  {
    if (local_costmap_->data[i] == -1)
    {
      relevant_obstacles_.data.push_back(-1);
    }
    else if (local_costmap_->data[i] < costmap_obstacle_treshold_)
    { // add treshold
      relevant_obstacles_.data.push_back(0);
    }
    else
    {

      // calculate cell in 2D space where robot is is point (0, 0)
      geometry_msgs::msg::Point cell;
      cell.x = static_cast<float>(i % local_costmap_->info.width)
          * local_costmap_->info.resolution
          + local_costmap_->info.origin.position.x;
      cell.y = static_cast<float>(i / local_costmap_->info.width)
          * local_costmap_->info.resolution
          + local_costmap_->info.origin.position.y;
      cell.z = 0.0f;

      cur_obstacle_relevant = false;
      cur_distance_to_center = getDistance2d(zero_position, cell);
      //check whether current obstacle lies inside the circumscribed_radius of the robot -> prevent collisions while rotating
      if (use_circumscribed && cur_distance_to_center <= circumscribed_radius)
      {
        cur_obstacle_robot = cell;

        if (obstacleValid(cur_obstacle_robot.x, cur_obstacle_robot.y))
        {
          cur_obstacle_relevant = true;
          obstacle_theta_robot = atan2(cur_obstacle_robot.y, cur_obstacle_robot.x);
        }

        //for each obstacle, now check whether it lies in the tube or not:
      }
      else if (use_tube && cur_distance_to_center < influence_radius_)
      {
        cur_obstacle_robot = cell;

        if (obstacleValid(cur_obstacle_robot.x, cur_obstacle_robot.y))
        {
          obstacle_theta_robot = atan2(cur_obstacle_robot.y, cur_obstacle_robot.x);
          obstacle_delta_theta_robot = obstacle_theta_robot - velocity_angle;
          obstacle_dist_vel_dir = sin(obstacle_delta_theta_robot) * cur_distance_to_center;

          if (obstacle_dist_vel_dir <= tube_left_border && obstacle_dist_vel_dir >= tube_right_border)
          {
            //found obstacle that lies inside of observation tube

            if (sign(obstacle_dist_vel_dir) >= 0)
            {
              if (cos(obstacle_delta_theta_robot) * cur_distance_to_center >= tube_left_origin)
              {
                //relevant obstacle in tube found
                cur_obstacle_relevant = true;
              }
            }
            else
            { // obstacle in right part of tube
              if (cos(obstacle_delta_theta_robot) * cur_distance_to_center >= tube_right_origin)
              {
                //relevant obstacle in tube found
                cur_obstacle_relevant = true;
              }
            }
          }
        }
      }

      if (cur_obstacle_relevant)
      {
        //TODO: switch to info
        RCLCPP_INFO(this->get_logger(), "Obstacle detected.");
        //relevant obstacle in tube found
        relevant_obstacles_.data.push_back(100);

        //now calculate distance of current, relevant obstacle to robot
        if (obstacle_theta_robot >= corner_front_right && obstacle_theta_robot < corner_front_left)
        {
          //obstacle in front:
          cur_distance_to_border = cur_distance_to_center - fabs(footprint_front_) / fabs(cos(obstacle_theta_robot));
        }
        else if (obstacle_theta_robot >= corner_front_left && obstacle_theta_robot < corner_rear_left)
        {
          //obstacle left:
          cur_distance_to_border = cur_distance_to_center - fabs(footprint_left_) / fabs(sin(obstacle_theta_robot));
        }
        else if (obstacle_theta_robot >= corner_rear_left || obstacle_theta_robot < corner_rear_right)
        {
          //obstacle in rear:
          cur_distance_to_border = cur_distance_to_center - fabs(footprint_rear_) / fabs(cos(obstacle_theta_robot));
        }
        else
        {
          //obstacle right:
          cur_distance_to_border = cur_distance_to_center - fabs(footprint_right_) / fabs(sin(obstacle_theta_robot));
        }

        if (cur_distance_to_border < closest_obstacle_dist_)
        {
          closest_obstacle_dist_ = cur_distance_to_border;
          closest_obstacle_angle_ = obstacle_theta_robot;
        }
      }
      else
      {
        relevant_obstacles_.data.push_back(0);
      }
    }
  }

  topic_pub_relevant_obstacles_->publish(relevant_obstacles_);
  RCLCPP_INFO(this->get_logger(),
              "closest_obstacle_dist_ = %f.4", closest_obstacle_dist_);
}


double CollisionVelocityFilter::getDistance2d(geometry_msgs::msg::Point a, geometry_msgs::msg::Point b)
{
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

double CollisionVelocityFilter::sign(double x)
{
  if (x >= 0.0f)
    return 1.0f;
  else
    return -1.0f;
}

bool CollisionVelocityFilter::obstacleValid(double x_obstacle, double y_obstacle)
{
  if (x_obstacle < footprint_front_ && x_obstacle > footprint_rear_ && y_obstacle > footprint_right_
      && y_obstacle < footprint_left_)
  {
    RCLCPP_WARN(this->get_logger(), "Found an obstacle inside robot_footprint: Skip!");
    return false;
  }

  return true;
}

void CollisionVelocityFilter::stopMovement()
{
  geometry_msgs::msg::TwistStamped stop_twist;
  stop_twist.linear.x = 0.0f;
  stop_twist.linear.y = 0.0f;
  stop_twist.linear.z = 0.0f;
  stop_twist.angular.x = 0.0f;
  stop_twist.angular.y = 0.0f;
  stop_twist.linear.z = 0.0f;
  topic_pub_command_->publish(stop_twist);
  vx_last_ = 0.0;
  vy_last_ = 0.0;
  vtheta_last_ = 0.0;
}



//#######################
//#### main program ####
int main(int argc, char** argv)
{

  rclcpp::init(argc, argv);
  auto node = std::make_shared<CollisionVelocityFilter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
};
