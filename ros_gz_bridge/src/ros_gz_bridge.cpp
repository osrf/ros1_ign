// Copyright 2022 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ros_gz_bridge/ros_gz_bridge.hpp>

#include <memory>
#include <string>

#include "bridge_handle_ros_to_gz.hpp"
#include "bridge_handle_gz_to_ros.hpp"

#include <rclcpp/expand_topic_or_service_name.hpp>

namespace ros_gz_bridge
{

RosGzBridge::RosGzBridge(const rclcpp::NodeOptions & options)
: rclcpp::Node("ros_gz_bridge", options)
{
  gz_node_ = std::make_shared<gz::transport::Node>();

  this->declare_parameter<int>("subscription_heartbeat", 1000);
  this->declare_parameter<std::string>("config_file", "");
  this->declare_parameter<bool>("expand_gz_topic_names", false);
<<<<<<< HEAD
=======
  this->declare_parameter<bool>("override_timestamps_with_wall_time", false);
>>>>>>> 63b651a (Garden EOL (#662))

  int heartbeat;
  this->get_parameter("subscription_heartbeat", heartbeat);
  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(heartbeat),
    std::bind(&RosGzBridge::spin, this));
}

void RosGzBridge::spin()
{
  if (handles_.empty()) {
    std::string config_file;
    this->get_parameter("config_file", config_file);
    bool expand_names;
    this->get_parameter("expand_gz_topic_names", expand_names);
    const std::string ros_ns = this->get_namespace();
    const std::string ros_node_name = this->get_name();
    if (!config_file.empty()) {
      auto entries = readFromYamlFile(config_file);
      for (auto & entry : entries) {
        if (expand_names) {
          entry.gz_topic_name = rclcpp::expand_topic_or_service_name(
            entry.gz_topic_name, ros_node_name, ros_ns, false);
        }
        this->add_bridge(entry);
      }
    }
  }
  for (auto & bridge : handles_) {
    bridge->Spin();
  }
}

void RosGzBridge::add_bridge(const BridgeConfig & config)
{
  bool gz_to_ros = false;
  bool ros_to_gz = false;

  if (config.direction == BridgeDirection::GZ_TO_ROS) {
    gz_to_ros = true;
  }

  if (config.direction == BridgeDirection::ROS_TO_GZ) {
    ros_to_gz = true;
  }

  if (config.direction == BridgeDirection::BIDIRECTIONAL) {
    ros_to_gz = true;
    gz_to_ros = true;
  }

  try {
    if (gz_to_ros) {
      RCLCPP_INFO(
        this->get_logger(),
        "Creating GZ->ROS Bridge: [%s (%s) -> %s (%s)] (Lazy %d)",
        config.gz_topic_name.c_str(), config.gz_type_name.c_str(),
        config.ros_topic_name.c_str(), config.ros_type_name.c_str(),
        config.is_lazy);
      handles_.push_back(
        std::make_unique<ros_gz_bridge::BridgeHandleGzToRos>(
          shared_from_this(), gz_node_,
          config));

      handles_.back()->Start();
    }

    if (ros_to_gz) {
      RCLCPP_INFO(
        this->get_logger(),
        "Creating ROS->GZ Bridge: [%s (%s) -> %s (%s)] (Lazy %d)",
        config.ros_topic_name.c_str(), config.ros_type_name.c_str(),
        config.gz_topic_name.c_str(), config.gz_type_name.c_str(),
        config.is_lazy);
      handles_.push_back(
        std::make_unique<ros_gz_bridge::BridgeHandleRosToGz>(
          shared_from_this(), gz_node_,
          config));

      handles_.back()->Start();
    }
  } catch (std::runtime_error & _e) {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to create a bridge for topic [%s] with ROS2 type [%s] "
      "to topic [%s] with Gazebo Transport type [%s]",
      config.ros_topic_name.c_str(),
      config.ros_type_name.c_str(),
      config.gz_topic_name.c_str(),
      config.gz_type_name.c_str());
  }
}

void RosGzBridge::add_service_bridge(
  const std::string & ros_type_name,
  const std::string & gz_req_type_name,
  const std::string & gz_rep_type_name,
  const std::string & service_name)
{
  auto factory = get_service_factory(ros_type_name, gz_req_type_name, gz_rep_type_name);
  services_.push_back(factory->create_ros_service(shared_from_this(), gz_node_, service_name));
}

}  // namespace ros_gz_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros_gz_bridge::RosGzBridge)
