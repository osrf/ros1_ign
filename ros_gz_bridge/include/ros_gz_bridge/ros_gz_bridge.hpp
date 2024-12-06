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

#ifndef ROS_GZ_BRIDGE__ROS_GZ_BRIDGE_HPP_
#define ROS_GZ_BRIDGE__ROS_GZ_BRIDGE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <gz/msgs/config.hh>
#include <gz/transport/Node.hh>
#include <rclcpp/node.hpp>
#include "ros_gz_bridge/bridge_config.hpp"

<<<<<<< HEAD
// Dataframe is available from versions 8.4.0 (fortress) forward
// This can be removed when the minimum supported version passes 8.4.0
#if (IGNITION_MSGS_MAJOR_VERSION > 8) || \
  ((IGNITION_MSGS_MAJOR_VERSION == 8) && (IGNITION_MSGS_MINOR_VERSION >= 4))
#define HAVE_DATAFRAME true
#endif

#if (GZ_MSGS_MAJOR_VERSION > 8) || \
  ((GZ_MSGS_MAJOR_VERSION == 8) && (GZ_MSGS_MINOR_VERSION >= 4))
#define HAVE_DATAFRAME true
#endif

// MaterialColor is available from versions 10.1.0 (Harmonic) forward
// This can be removed when the minimum supported version passes 10.1.0
#if (GZ_MSGS_MAJOR_VERSION > 10) || \
  ((GZ_MSGS_MAJOR_VERSION == 10) && (GZ_MSGS_MINOR_VERSION >= 1))
#define HAVE_MATERIALCOLOR true
#endif

=======
>>>>>>> 63b651a (Garden EOL (#662))
namespace ros_gz_bridge
{
/// Forward declarations
class BridgeHandle;

/// \brief Component container for the ROS-GZ Bridge
class RosGzBridge : public rclcpp::Node
{
public:
  /// \brief Constructor
  /// \param[in] options options control creation of the ROS 2 node
  explicit RosGzBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

<<<<<<< HEAD
  /// \brief Add a new ROS-IGN bridge to the node
=======
  /// \brief Add a new ROS-GZ bridge to the node
>>>>>>> 63b651a (Garden EOL (#662))
  /// \param[in] config Parameters to control creation of a new bridge
  void add_bridge(const BridgeConfig & config);

  /// \brief Create a new ROS-GZ bridge for a service
  /// \param[in] ros_type_name Name of the ROS service (eg ros_bz_interfaces/srv/ControlWorld)
  /// \param[in] gz_req_type_name Gazebo service request type
  /// \param[in] gz_req_type_name Gazebo service response type
  /// \param[in] service_name Address of the service to be bridged
  void add_service_bridge(
    const std::string & ros_type_name,
    const std::string & gz_req_type_name,
    const std::string & gz_rep_type_name,
    const std::string & service_name);

protected:
  /// \brief Periodic callback to check connectivity and liveliness
  void spin();

protected:
  /// \brief Pointer to Gazebo node used to create publishers/subscribers
  std::shared_ptr<gz::transport::Node> gz_node_;

  /// \brief List of bridge handles
  std::vector<std::shared_ptr<ros_gz_bridge::BridgeHandle>> handles_;

  /// \brief List of bridged ROS services
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;

  /// \brief Timer to control periodic callback
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};
}  // namespace ros_gz_bridge

#endif  // ROS_GZ_BRIDGE__ROS_GZ_BRIDGE_HPP_
