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


#ifndef COB_FOOTPRINT_OBSERVER_H
#define COB_FOOTPRINT_OBSERVER_H

//##################
//#### includes ####

// ROS includes 

#include <rclcpp/rclcpp.hpp>

// message includes
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

/*
#include <footprint_observer/srv/get_footprint.hpp>


///
/// @class FootprintObserver
/// @brief checks the footprint of care-o-bot and advertises a service to get the adjusted footprint
///
///
class FootprintObserver : public rclcpp::Node
{
  public:
    ///
    /// @brief  constructor
    ///
    FootprintObserver();
    ///
    /// @brief  destructor
    ///
    ~FootprintObserver();

    ///
    /// @brief  checks the footprint of the robot if it needs to be enlarged due to arm or tray
    ///
    void checkFootprint();

    ///
    /// @brief  callback for GetFootprint service
    /// @param  req - request message to service
    /// @param  resp - response message from service
    /// @return service call successfull
    ///
    bool getFootprintCB(footprint_observer::srv::GetFootprint::Request &req, footprint_observer::srv::GetFootprint::Response &resp);

    // public members
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr topic_pub_footprint_;
    rclcpp::Service<footprint_observer::srv::GetFootprint>::SharedPtr srv_get_footprint_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  private:
    ///
    /// @brief  loads the robot footprint from the costmap node
    /// @param  node - costmap node to check for footprint parameter
    /// @return points of a polygon specifying the footprint
    ///
    std::vector<geometry_msgs::msg::Point> loadRobotFootprint(ros::NodeHandle node);

    ///
    /// @brief  publishes the adjusted footprint as geometry_msgs::StampedPolygon message
    ///
    void publishFootprint();

    ///
    /// @brief  computes the sign of x
    /// @param  x - number
    /// @return sign of x
    ///
    double sign(double x);

    // private members
    std::vector<geometry_msgs::msg::Point> robot_footprint_;
    double epsilon_;
    double footprint_front_initial_, footprint_rear_initial_, footprint_left_initial_, footprint_right_initial_;
    double footprint_front_, footprint_rear_, footprint_left_, footprint_right_;
    std::string frames_to_check_;
    std::string robot_base_frame_;

    pthread_mutex_t m_mutex;

    rclcpp::Time last_tf_missing_;
    unsigned int times_warned_;
};
*/

#endif
