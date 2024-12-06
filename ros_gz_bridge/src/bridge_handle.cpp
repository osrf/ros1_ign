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
#include "bridge_handle.hpp"

#include <memory>
#include <string>

#include "get_factory.hpp"

namespace ros_gz_bridge
{
BridgeHandle::BridgeHandle(
  rclcpp::Node::SharedPtr ros_node,
  std::shared_ptr<gz::transport::Node> gz_node,
  const BridgeConfig & config)
: ros_node_(ros_node),
  gz_node_(gz_node),
  config_(config),
  factory_(get_factory(config.ros_type_name, config.gz_type_name))
{
<<<<<<< HEAD
=======
  ros_node_->get_parameter("override_timestamps_with_wall_time",
    override_timestamps_with_wall_time_);
>>>>>>> 63b651a (Garden EOL (#662))
}

BridgeHandle::~BridgeHandle() = default;

bool BridgeHandle::IsLazy() const
{
  return config_.is_lazy;
}

void BridgeHandle::Start()
{
  if (!this->HasPublisher()) {
    this->StartPublisher();
  }

  if (!this->IsLazy() && !this->HasSubscriber()) {
    this->StartSubscriber();
  }
}

void BridgeHandle::Spin()
{
  if (!this->IsLazy()) {
    return;
  }

  if (this->HasSubscriber() && this->NumSubscriptions() == 0) {
    RCLCPP_DEBUG(
      this->ros_node_->get_logger(),
      "Bridge [%s] - No subscriptions found, stopping bridge",
      config_.ros_topic_name.c_str());
    this->StopSubscriber();
  } else if (!this->HasSubscriber() && this->NumSubscriptions() > 0) {
    RCLCPP_DEBUG(
      this->ros_node_->get_logger(),
      "Bridge [%s] - Subscriptions found, starting bridge",
      config_.ros_topic_name.c_str());
    this->StartSubscriber();
  }
}
}  // namespace ros_gz_bridge
