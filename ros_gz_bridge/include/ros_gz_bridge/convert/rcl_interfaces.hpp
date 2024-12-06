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

#ifndef ROS_GZ_BRIDGE__CONVERT__RCL_INTERFACES_HPP_
#define ROS_GZ_BRIDGE__CONVERT__RCL_INTERFACES_HPP_

<<<<<<< HEAD
// Ignition messages
=======
// GZ messages
>>>>>>> 63b651a (Garden EOL (#662))
#include <gz/msgs/any.pb.h>

// ROS 2 messages
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>

#include <ros_gz_bridge/convert_decl.hpp>

namespace ros_gz_bridge
{

template<>
void
convert_ros_to_gz(
  const rcl_interfaces::msg::ParameterValue & ros_msg,
  gz::msgs::Any & ign_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Any & ign_msg,
  rcl_interfaces::msg::ParameterValue & ros_msg);

}  // namespace ros_gz_bridge
#endif  // ROS_GZ_BRIDGE__CONVERT__RCL_INTERFACES_HPP_
