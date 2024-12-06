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

#ifndef ROS_GZ_BRIDGE__CONVERT__GEOMETRY_MSGS_HPP_
#define ROS_GZ_BRIDGE__CONVERT__GEOMETRY_MSGS_HPP_

// Gazebo Msgs
#include <gz/msgs/quaternion.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_with_covariance.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/twist_with_covariance.pb.h>
#include <gz/msgs/wrench.pb.h>

// ROS 2 messages
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
<<<<<<< HEAD
=======
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
>>>>>>> 63b651a (Garden EOL (#662))
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <ros_gz_bridge/convert_decl.hpp>

namespace ros_gz_bridge
{

// geometry_msgs
template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::Quaternion & ros_msg,
  gz::msgs::Quaternion & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Quaternion & gz_msg,
  geometry_msgs::msg::Quaternion & ros_msg);

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::Vector3 & ros_msg,
  gz::msgs::Vector3d & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Vector3d & gz_msg,
  geometry_msgs::msg::Vector3 & ros_msg);

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::Point & ros_msg,
  gz::msgs::Vector3d & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Vector3d & gz_msg,
  geometry_msgs::msg::Point & ros_msg);

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::Pose & ros_msg,
  gz::msgs::Pose & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Pose & gz_msg,
  geometry_msgs::msg::Pose & ros_msg);

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::PoseArray & ros_msg,
  gz::msgs::Pose_V & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Pose_V & gz_msg,
  geometry_msgs::msg::PoseArray & ros_msg);

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::PoseWithCovariance & ros_msg,
  gz::msgs::PoseWithCovariance & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::PoseWithCovariance & gz_msg,
<<<<<<< HEAD
=======
  geometry_msgs::msg::PoseWithCovarianceStamped & ros_msg);

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::PoseWithCovarianceStamped & ros_msg,
  gz::msgs::PoseWithCovariance & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::PoseWithCovariance & gz_msg,
>>>>>>> 63b651a (Garden EOL (#662))
  geometry_msgs::msg::PoseWithCovariance & ros_msg);

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::PoseStamped & ros_msg,
  gz::msgs::Pose & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Pose & gz_msg,
  geometry_msgs::msg::PoseStamped & ros_msg);

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::Transform & ros_msg,
  gz::msgs::Pose & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Pose & gz_msg,
  geometry_msgs::msg::Transform & ros_msg);

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::TransformStamped & ros_msg,
  gz::msgs::Pose & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Pose & gz_msg,
  geometry_msgs::msg::TransformStamped & ros_msg);

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::Twist & ros_msg,
  gz::msgs::Twist & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Twist & gz_msg,
  geometry_msgs::msg::Twist & ros_msg);

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::TwistStamped & ros_msg,
  gz::msgs::Twist & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Twist & gz_msg,
  geometry_msgs::msg::TwistStamped & ros_msg);

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::TwistWithCovariance & ros_msg,
  gz::msgs::TwistWithCovariance & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::TwistWithCovariance & gz_msg,
  geometry_msgs::msg::TwistWithCovariance & ros_msg);

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::TwistWithCovarianceStamped & ros_msg,
  gz::msgs::TwistWithCovariance & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::TwistWithCovariance & gz_msg,
  geometry_msgs::msg::TwistWithCovarianceStamped & ros_msg);

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::Wrench & ros_msg,
  gz::msgs::Wrench & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Wrench & gz_msg,
  geometry_msgs::msg::Wrench & ros_msg);

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::WrenchStamped & ros_msg,
  gz::msgs::Wrench & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Wrench & gz_msg,
  geometry_msgs::msg::WrenchStamped & ros_msg);

}  // namespace ros_gz_bridge

#endif  // ROS_GZ_BRIDGE__CONVERT__GEOMETRY_MSGS_HPP_
