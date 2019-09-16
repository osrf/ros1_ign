// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef ROS_IGN_BRIDGE__BRIDGE_HPP_
#define ROS_IGN_BRIDGE__BRIDGE_HPP_

// include Ignition Transport
#include <ignition/transport/Node.hh>

#include <ros_ign_bridge/builtin_interfaces_factories.hpp>

#include <memory>
#include <string>

namespace ros_ign_bridge
{

struct BridgeRostoIgnHandles
{
  rclcpp::SubscriptionBase::SharedPtr ros_subscriber;
  ignition::transport::Node::Publisher ign_publisher;
};

struct BridgeIgntoRosHandles
{
  std::shared_ptr<ignition::transport::Node> ign_subscriber;
  rclcpp::PublisherBase::SharedPtr ros_publisher;
};

struct BridgeHandles
{
  BridgeRostoIgnHandles bridgeRostoIgn;
  BridgeIgntoRosHandles bridgeIgntoRos;
};

BridgeRostoIgnHandles
create_bridge_from_ros_to_ign(
  rclcpp::Node::SharedPtr ros_node,
  std::shared_ptr<ignition::transport::Node> ign_node,
  const std::string & ros_type_name,
  const std::string & ros_topic_name,
  const std::string & ign_type_name,
  const std::string & ign_topic_name,
  size_t queue_size)
{
  auto factory = get_factory(ros_type_name, ign_type_name);
  auto ign_pub = factory->create_ign_publisher(ign_node, ign_topic_name);

  auto ros_sub = factory->create_ros_subscriber(ros_node, ros_topic_name, queue_size, ign_pub);

  BridgeRostoIgnHandles handles;
  handles.ros_subscriber = ros_sub;
  handles.ign_publisher = ign_pub;
  return handles;
}

BridgeIgntoRosHandles
create_bridge_from_ign_to_ros(
  std::shared_ptr<ignition::transport::Node> ign_node,
  rclcpp::Node::SharedPtr ros_node,
  const std::string & ign_type_name,
  const std::string & ign_topic_name,
  const std::string & ros_type_name,
  const std::string & ros_topic_name,
  size_t queue_size)
{
  auto factory = get_factory(ros_type_name, ign_type_name);
  auto ros_pub = factory->create_ros_publisher(ros_node, ros_topic_name, queue_size);

  factory->create_ign_subscriber(ign_node, ign_topic_name, ros_pub);

  BridgeIgntoRosHandles handles;
  handles.ign_subscriber = ign_node;
  handles.ros_publisher = ros_pub;
  return handles;
}

BridgeHandles
create_bidirectional_bridge(
  rclcpp::Node::SharedPtr ros_node,
  std::shared_ptr<ignition::transport::Node> ign_node,
  const std::string & ros_type_name,
  const std::string & ign_type_name,
  const std::string & topic_name,
  size_t queue_size = 10)
{
  BridgeHandles handles;
  handles.bridgeRostoIgn = create_bridge_from_ros_to_ign(
    ros_node, ign_node,
    ros_type_name, topic_name, ign_type_name, topic_name, queue_size);
  handles.bridgeIgntoRos = create_bridge_from_ign_to_ros(
    ign_node, ros_node,
    ign_type_name, topic_name, ros_type_name, topic_name, queue_size);
  return handles;
}

}  // namespace ros_ign_bridge

#endif  // ROS_IGN_BRIDGE__BRIDGE_HPP_
