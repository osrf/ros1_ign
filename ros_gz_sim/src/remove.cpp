// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

#include <string>
#include <optional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <gz/transport/Node.hh>


constexpr unsigned int timeout{5000};
std::optional<std::string> retrieve_world_name(const rclcpp::Node::SharedPtr & ros2_node)
{
  gz::transport::Node node;
  bool executed{};
  bool result{};
  const std::string service{"/gazebo/worlds"};
  gz::msgs::StringMsg_V worlds_msg;

    // This loop is here to allow the server time to download resources.
  while (rclcpp::ok() && !executed) {
    RCLCPP_INFO(ros2_node->get_logger(), "Requesting list of world names.");
    executed = node.Request(service, timeout, worlds_msg, result);
  }

  if (!executed) {
    RCLCPP_INFO(ros2_node->get_logger(), "Timed out when getting world names.");
    return std::nullopt;
  }

  if (!result || worlds_msg.data().empty()) {
    RCLCPP_INFO(ros2_node->get_logger(), "Failed to get world names.");
    return std::nullopt;
  }

  return worlds_msg.data(0);
}

int remove_entity(
  const std::string & entity_name,
  const std::string & world_name,
  const rclcpp::Node::SharedPtr & ros2_node)
{
  const std::string service{"/world/" + world_name + "/remove"};
  gz::msgs::Entity entity_remove_request;
  entity_remove_request.set_name(entity_name);
  entity_remove_request.set_type(gz::msgs::Entity_Type_MODEL);
  gz::transport::Node node;
  gz::msgs::Boolean response;
  bool result;

  while(rclcpp::ok() && !node.Request(service, entity_remove_request, timeout, response, result)) {
    RCLCPP_WARN(
            ros2_node->get_logger(), "Waiting for service [%s] to become available ...",
            service.c_str());
  }
  if (result && response.data()) {
    RCLCPP_INFO(ros2_node->get_logger(), "Entity removal successful.");
    return 0;
  } else {
    RCLCPP_ERROR(
            ros2_node->get_logger(), "Entity removal failed.\n %s",
            entity_remove_request.DebugString().c_str());
    return 1;
  }
  RCLCPP_INFO(ros2_node->get_logger(), "Entity removal was interrupted.");
  return 1;
}

int main(int _argc, char ** _argv)
{
  rclcpp::init(_argc, _argv);
  auto ros2_node = rclcpp::Node::make_shared("ros_gz_sim_remove_entity");
  ros2_node->declare_parameter("world", "");
  ros2_node->declare_parameter("entity_name", "");

  std::string world_name = ros2_node->get_parameter("world").as_string();
  if (world_name.empty()) {
    world_name = retrieve_world_name(ros2_node).value_or("");
    if (world_name.empty()) {
      return -1;
    }
  }

  const std::string entity_name =
    ros2_node->get_parameter("entity_name").as_string();
  if (entity_name.empty()) {
    RCLCPP_INFO(ros2_node->get_logger(),
            "Entity to remove name is not provided, entity will not be removed.");
    return -1;
  }

  return remove_entity(entity_name, world_name, ros2_node);
}
