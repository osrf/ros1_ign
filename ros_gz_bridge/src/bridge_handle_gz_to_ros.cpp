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
//
#include <memory>

#include "bridge_handle_gz_to_ros.hpp"

namespace ros_gz_bridge
{

BridgeHandleGzToRos::~BridgeHandleGzToRos() = default;

size_t BridgeHandleGzToRos::NumSubscriptions() const
{
  // Return number of ROS subscriptions
  size_t valid_subscriptions = 0;

  if (this->ros_publisher_ != nullptr) {
    // Use info_by_topic rather than get_subscription_count
    // to filter out potential bidirectional bridge
    auto topic_info = this->ros_node_->get_subscriptions_info_by_topic(
      this->config_.ros_topic_name);

    for (auto & topic : topic_info) {
      if (topic.node_name() == this->ros_node_->get_name()) {
        continue;
      }
      valid_subscriptions++;
    }
  }

  return valid_subscriptions;
}

bool BridgeHandleGzToRos::HasPublisher() const
{
  return this->ros_publisher_ != nullptr;
}

void BridgeHandleGzToRos::StartPublisher()
{
  // Start ROS publisher
  this->ros_publisher_ = this->factory_->create_ros_publisher(
    this->ros_node_,
    this->config_.ros_topic_name,
    this->config_.publisher_queue_size);
}

bool BridgeHandleGzToRos::HasSubscriber() const
{
  // Return Gazebo subscriber status
  return this->gz_subscriber_ != nullptr;
}

void BridgeHandleGzToRos::StartSubscriber()
{
  // Start Gazebo subscriber
  this->factory_->create_gz_subscriber(
    this->gz_node_,
    this->config_.gz_topic_name,
    this->config_.subscriber_queue_size,
<<<<<<< HEAD
    this->ros_publisher_);
=======
    this->ros_publisher_,
    override_timestamps_with_wall_time_);
>>>>>>> 63b651a (Garden EOL (#662))

  this->gz_subscriber_ = this->gz_node_;
}

void BridgeHandleGzToRos::StopSubscriber()
{
  // Stop Gazebo subscriber
  if (!this->gz_subscriber_) {
    return;
  }

  this->gz_subscriber_->Unsubscribe(this->config_.gz_topic_name);
  this->gz_subscriber_.reset();
}

}  // namespace ros_gz_bridge
