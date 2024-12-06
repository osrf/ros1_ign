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

#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <vector>

// include ROS 2
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/ros_gz_bridge.hpp>

#include "bridge_handle.hpp"

//////////////////////////////////////////////////
void usage()
{
  std::cout << "Bridge a collection of ROS2 and Gazebo Transport topics and services.\n\n" <<
    "  parameter_bridge [<topic@ROS2_type@Ign_type> ..] " <<
    " [<service@ROS2_srv_type[@Ign_req_type@Ign_rep_type]> ..]\n\n" <<
    "Topics: The first @ symbol delimits the topic name from the message types.\n" <<
    "Following the first @ symbol is the ROS message type.\n" <<
    "The ROS message type is followed by an @, [, or ] symbol where\n" <<
    "    @  == a bidirectional bridge, \n" <<
    "    [  == a bridge from Gazebo to ROS,\n" <<
    "    ]  == a bridge from ROS to Gazebo.\n" <<
    "Following the direction symbol is the Gazebo Transport message " <<
    "type.\n\n" <<
    "Services: The first @ symbol delimits the service name from the types.\n" <<
    "Following the first @ symbol is the ROS service type.\n" <<
    "Optionally, you can include the Gazebo request and response type\n" <<
    "separated by the @ symbol.\n" <<
    "It is only supported to expose Gazebo servces as ROS services, i.e.\n"
    "the ROS service will forward request to the Gazebo service and then forward\n"
    "the response back to the ROS client.\n\n"
    "A bidirectional bridge example:\n" <<
<<<<<<< HEAD
    "    parameter_bridge /chatter@std_msgs/String@ignition.msgs" <<
    ".StringMsg\n\n" <<
    "A bridge from Gazebo to ROS example:\n" <<
    "    parameter_bridge /chatter@std_msgs/String[ignition.msgs" <<
    ".StringMsg\n\n" <<
    "A bridge from ROS to Gazebo example:\n" <<
    "    parameter_bridge /chatter@std_msgs/String]ignition.msgs" <<
=======
    "    parameter_bridge /chatter@std_msgs/String@gz.msgs" <<
    ".StringMsg\n\n" <<
    "A bridge from Gazebo to ROS example:\n" <<
    "    parameter_bridge /chatter@std_msgs/String[gz.msgs" <<
    ".StringMsg\n\n" <<
    "A bridge from ROS to Gazebo example:\n" <<
    "    parameter_bridge /chatter@std_msgs/String]gz.msgs" <<
>>>>>>> 63b651a (Garden EOL (#662))
    ".StringMsg\n" <<
    "A service bridge:\n" <<
    "    parameter_bridge /world/default/control@ros_gz_interfaces/srv/ControlWorld\n" <<
    "Or equivalently:\n" <<
    "    parameter_bridge /world/default/control@ros_gz_interfaces/srv/ControlWorld@"
<<<<<<< HEAD
    "ignition.msgs.WorldControl@ignition.msgs.Boolean\n" << std::endl;
=======
    "gz.msgs.WorldControl@gz.msgs.Boolean\n" << std::endl;
>>>>>>> 63b651a (Garden EOL (#662))
}

using RosGzBridge = ros_gz_bridge::RosGzBridge;
using BridgeDirection = ros_gz_bridge::BridgeDirection;
using BridgeConfig = ros_gz_bridge::BridgeConfig;

//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  if (argc < 2) {
    usage();
    return -1;
  }
  // skip the process name in argument procesing
  ++argv;
  --argc;
  auto filteredArgs = rclcpp::init_and_remove_ros_arguments(argc, argv);

  auto bridge_node = std::make_shared<RosGzBridge>(rclcpp::NodeOptions());

  // Set lazy subscriber on a global basis
  bool lazy_subscription = false;
  bridge_node->declare_parameter<bool>("lazy", false);
  bridge_node->get_parameter("lazy", lazy_subscription);

  const std::string delim = "@";
  const std::string delimGzToROS = "[";
  const std::string delimROSToGz = "]";

  // TODO(ivanpauno): Improve the parsing code later, it's hard to read ...
  for (auto filteredArg : filteredArgs) {
    std::string arg = filteredArg;
    auto delimPos = arg.find(delim);
    if (delimPos == std::string::npos || delimPos == 0) {
      usage();
      return -1;
    }

    BridgeConfig config;
    config.ros_topic_name = arg.substr(0, delimPos);
    config.gz_topic_name = arg.substr(0, delimPos);
    arg.erase(0, delimPos + delim.size());

    // Get the direction delimeter, which should be one of:
    //   @ == bidirectional, or
    //   [ == only from GZ to ROS, or
    //   ] == only from ROS to GZ.
    delimPos = arg.find(delim);
    config.direction = BridgeDirection::BIDIRECTIONAL;

    if (delimPos == std::string::npos || delimPos == 0) {
      delimPos = arg.find(delimGzToROS);
      if (delimPos == std::string::npos || delimPos == 0) {
        delimPos = arg.find(delimROSToGz);
        if (delimPos == 0) {
          usage();
          return -1;
        } else if (delimPos == std::string::npos) {
          // Fall through, to parse for services
          config.direction = BridgeDirection::NONE;
        } else {
          config.direction = BridgeDirection::ROS_TO_GZ;
        }
      } else {
        config.direction = BridgeDirection::GZ_TO_ROS;
      }
    }

    config.ros_type_name = arg.substr(0, delimPos);
    arg.erase(0, delimPos + delim.size());
    if (config.ros_type_name.find("/srv/") != std::string::npos) {
      std::string gz_req_type_name;
      std::string gz_rep_type_name;
      if (config.direction == BridgeDirection::ROS_TO_GZ ||
        config.direction == BridgeDirection::GZ_TO_ROS)
      {
        usage();
        return -1;
      }
      if (config.direction == BridgeDirection::BIDIRECTIONAL) {
        delimPos = arg.find(delim);
        if (delimPos == std::string::npos || delimPos == 0) {
          usage();
          return -1;
        }
        gz_req_type_name = arg.substr(0, delimPos);
        arg.erase(0, delimPos + delim.size());
        gz_rep_type_name = std::move(arg);
      }
      try {
        bridge_node->add_service_bridge(
          config.ros_type_name,
          gz_req_type_name,
          gz_rep_type_name,
          config.ros_topic_name);
      } catch (std::runtime_error & e) {
        std::cerr << e.what() << std::endl;
      }
      continue;
    }

    delimPos = arg.find(delim);
    if (delimPos != std::string::npos || arg.empty()) {
      usage();
      return -1;
    }
    config.gz_type_name = arg;
    bridge_node->add_bridge(config);
  }

  // ROS 2 spinner
  rclcpp::spin(bridge_node);

  return 0;
}
