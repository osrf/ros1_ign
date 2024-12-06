// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef  FACTORY_INTERFACE_HPP_
#define  FACTORY_INTERFACE_HPP_

#include <memory>
#include <string>

// include Gazebo Transport
#include <gz/transport/Node.hh>

// include ROS 2
#include <rclcpp/rclcpp.hpp>

namespace ros_gz_bridge
{

class FactoryInterface
{
public:
  virtual ~FactoryInterface() = 0;

  virtual
  rclcpp::PublisherBase::SharedPtr
  create_ros_publisher(
    rclcpp::Node::SharedPtr ros_node,
    const std::string & topic_name,
    size_t queue_size) = 0;

  virtual
  gz::transport::Node::Publisher
  create_gz_publisher(
    std::shared_ptr<gz::transport::Node> gz_node,
    const std::string & topic_name,
    size_t queue_size) = 0;

  virtual
  rclcpp::SubscriptionBase::SharedPtr
  create_ros_subscriber(
    rclcpp::Node::SharedPtr ros_node,
    const std::string & topic_name,
    size_t queue_size,
    gz::transport::Node::Publisher & gz_pub) = 0;

  virtual
  void
  create_gz_subscriber(
    std::shared_ptr<gz::transport::Node> node,
    const std::string & topic_name,
    size_t queue_size,
<<<<<<< HEAD
    rclcpp::PublisherBase::SharedPtr ros_pub) = 0;
=======
    rclcpp::PublisherBase::SharedPtr ros_pub,
    bool override_timestamps_with_wall_time) = 0;
>>>>>>> 63b651a (Garden EOL (#662))
};

}  // namespace ros_gz_bridge

#endif  // FACTORY_INTERFACE_HPP_
