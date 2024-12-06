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

#include "convert/utils.hpp"
#include "ros_gz_bridge/convert/geometry_msgs.hpp"

namespace ros_gz_bridge
{

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::Quaternion & ros_msg,
  gz::msgs::Quaternion & gz_msg)
{
  gz_msg.set_x(ros_msg.x);
  gz_msg.set_y(ros_msg.y);
  gz_msg.set_z(ros_msg.z);
  gz_msg.set_w(ros_msg.w);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Quaternion & gz_msg,
  geometry_msgs::msg::Quaternion & ros_msg)
{
  ros_msg.x = gz_msg.x();
  ros_msg.y = gz_msg.y();
  ros_msg.z = gz_msg.z();
  ros_msg.w = gz_msg.w();
}

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::Vector3 & ros_msg,
  gz::msgs::Vector3d & gz_msg)
{
  gz_msg.set_x(ros_msg.x);
  gz_msg.set_y(ros_msg.y);
  gz_msg.set_z(ros_msg.z);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Vector3d & gz_msg,
  geometry_msgs::msg::Vector3 & ros_msg)
{
  ros_msg.x = gz_msg.x();
  ros_msg.y = gz_msg.y();
  ros_msg.z = gz_msg.z();
}

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::Point & ros_msg,
  gz::msgs::Vector3d & gz_msg)
{
  gz_msg.set_x(ros_msg.x);
  gz_msg.set_y(ros_msg.y);
  gz_msg.set_z(ros_msg.z);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Vector3d & gz_msg,
  geometry_msgs::msg::Point & ros_msg)
{
  ros_msg.x = gz_msg.x();
  ros_msg.y = gz_msg.y();
  ros_msg.z = gz_msg.z();
}

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::Pose & ros_msg,
  gz::msgs::Pose & gz_msg)
{
  convert_ros_to_gz(ros_msg.position, *gz_msg.mutable_position());
  convert_ros_to_gz(ros_msg.orientation, *gz_msg.mutable_orientation());
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Pose & gz_msg,
  geometry_msgs::msg::Pose & ros_msg)
{
  convert_gz_to_ros(gz_msg.position(), ros_msg.position);
  convert_gz_to_ros(gz_msg.orientation(), ros_msg.orientation);
}

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::PoseArray & ros_msg,
  gz::msgs::Pose_V & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  gz_msg.clear_pose();
  for (auto const & t : ros_msg.poses) {
    auto p = gz_msg.add_pose();
    convert_ros_to_gz(t, *p);
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Pose_V & gz_msg,
  geometry_msgs::msg::PoseArray & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  ros_msg.poses.clear();
  for (auto const & p : gz_msg.pose()) {
    geometry_msgs::msg::Pose pose;
    convert_gz_to_ros(p, pose);
    ros_msg.poses.push_back(pose);
  }
}

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::PoseWithCovariance & ros_msg,
  gz::msgs::PoseWithCovariance & gz_msg)
{
  convert_ros_to_gz(ros_msg.pose.position, *gz_msg.mutable_pose()->mutable_position());
  convert_ros_to_gz(ros_msg.pose.orientation, *gz_msg.mutable_pose()->mutable_orientation());
  for (const auto & elem : ros_msg.covariance) {
    gz_msg.mutable_covariance()->add_data(elem);
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::PoseWithCovariance & gz_msg,
  geometry_msgs::msg::PoseWithCovariance & ros_msg)
{
  convert_gz_to_ros(gz_msg.pose().position(), ros_msg.pose.position);
  convert_gz_to_ros(gz_msg.pose().orientation(), ros_msg.pose.orientation);
  int data_size = gz_msg.covariance().data_size();
  if (data_size == 36) {
    for (int i = 0; i < data_size; i++) {
      auto data = gz_msg.covariance().data()[i];
      ros_msg.covariance[i] = data;
    }
  }
}

template<>
void
convert_ros_to_gz(
<<<<<<< HEAD
=======
  const geometry_msgs::msg::PoseWithCovarianceStamped & ros_msg,
  gz::msgs::PoseWithCovariance & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_pose()->mutable_header()));
  convert_ros_to_gz(ros_msg.pose, gz_msg);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::PoseWithCovariance & gz_msg,
  geometry_msgs::msg::PoseWithCovarianceStamped & ros_msg)
{
  convert_gz_to_ros(gz_msg.pose().header(), ros_msg.header);
  convert_gz_to_ros(gz_msg, ros_msg.pose);
}


template<>
void
convert_ros_to_gz(
>>>>>>> 63b651a (Garden EOL (#662))
  const geometry_msgs::msg::PoseStamped & ros_msg,
  gz::msgs::Pose & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  convert_ros_to_gz(ros_msg.pose, gz_msg);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Pose & gz_msg,
  geometry_msgs::msg::PoseStamped & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  convert_gz_to_ros(gz_msg, ros_msg.pose);
}

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::Transform & ros_msg,
  gz::msgs::Pose & gz_msg)
{
  convert_ros_to_gz(ros_msg.translation, *gz_msg.mutable_position());
  convert_ros_to_gz(ros_msg.rotation, *gz_msg.mutable_orientation());
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Pose & gz_msg,
  geometry_msgs::msg::Transform & ros_msg)
{
  convert_gz_to_ros(gz_msg.position(), ros_msg.translation);
  convert_gz_to_ros(gz_msg.orientation(), ros_msg.rotation);
}

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::TransformStamped & ros_msg,
  gz::msgs::Pose & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  convert_ros_to_gz(ros_msg.transform, gz_msg);

  auto newPair = gz_msg.mutable_header()->add_data();
  newPair->set_key("child_frame_id");
  newPair->add_value(ros_msg.child_frame_id);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Pose & gz_msg,
  geometry_msgs::msg::TransformStamped & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  convert_gz_to_ros(gz_msg, ros_msg.transform);
  for (auto i = 0; i < gz_msg.header().data_size(); ++i) {
    auto aPair = gz_msg.header().data(i);
    if (aPair.key() == "child_frame_id" && aPair.value_size() > 0) {
      ros_msg.child_frame_id = frame_id_gz_to_ros(aPair.value(0));
      break;
    }
  }
}

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::Twist & ros_msg,
  gz::msgs::Twist & gz_msg)
{
  convert_ros_to_gz(ros_msg.linear, (*gz_msg.mutable_linear()));
  convert_ros_to_gz(ros_msg.angular, (*gz_msg.mutable_angular()));
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Twist & gz_msg,
  geometry_msgs::msg::Twist & ros_msg)
{
  convert_gz_to_ros(gz_msg.linear(), ros_msg.linear);
  convert_gz_to_ros(gz_msg.angular(), ros_msg.angular);
}

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::TwistStamped & ros_msg,
  gz::msgs::Twist & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  convert_ros_to_gz(ros_msg.twist, gz_msg);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Twist & gz_msg,
  geometry_msgs::msg::TwistStamped & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  convert_gz_to_ros(gz_msg, ros_msg.twist);
}

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::TwistWithCovariance & ros_msg,
  gz::msgs::TwistWithCovariance & gz_msg)
{
  convert_ros_to_gz(ros_msg.twist.linear, (*gz_msg.mutable_twist()->mutable_linear()));
  convert_ros_to_gz(ros_msg.twist.angular, (*gz_msg.mutable_twist()->mutable_angular()));
  for (const auto & elem : ros_msg.covariance) {
    gz_msg.mutable_covariance()->add_data(elem);
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::TwistWithCovariance & gz_msg,
  geometry_msgs::msg::TwistWithCovariance & ros_msg)
{
  convert_gz_to_ros(gz_msg.twist().linear(), ros_msg.twist.linear);
  convert_gz_to_ros(gz_msg.twist().angular(), ros_msg.twist.angular);
  int data_size = gz_msg.covariance().data_size();
  if (data_size == 36) {
    for (int i = 0; i < data_size; i++) {
      auto data = gz_msg.covariance().data()[i];
      ros_msg.covariance[i] = data;
    }
  }
}

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::TwistWithCovarianceStamped & ros_msg,
  gz::msgs::TwistWithCovariance & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_twist()->mutable_header()));
  convert_ros_to_gz(ros_msg.twist, gz_msg);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::TwistWithCovariance & gz_msg,
  geometry_msgs::msg::TwistWithCovarianceStamped & ros_msg)
{
  convert_gz_to_ros(gz_msg.twist().header(), ros_msg.header);
  convert_gz_to_ros(gz_msg, ros_msg.twist);
}

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::Wrench & ros_msg,
  gz::msgs::Wrench & gz_msg)
{
  convert_ros_to_gz(ros_msg.force, (*gz_msg.mutable_force()));
  convert_ros_to_gz(ros_msg.torque, (*gz_msg.mutable_torque()));
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Wrench & gz_msg,
  geometry_msgs::msg::Wrench & ros_msg)
{
  convert_gz_to_ros(gz_msg.force(), ros_msg.force);
  convert_gz_to_ros(gz_msg.torque(), ros_msg.torque);
}

template<>
void
convert_ros_to_gz(
  const geometry_msgs::msg::WrenchStamped & ros_msg,
  gz::msgs::Wrench & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  convert_ros_to_gz(ros_msg.wrench.force, (*gz_msg.mutable_force()));
  convert_ros_to_gz(ros_msg.wrench.torque, (*gz_msg.mutable_torque()));
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Wrench & gz_msg,
  geometry_msgs::msg::WrenchStamped & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  convert_gz_to_ros(gz_msg.force(), ros_msg.wrench.force);
  convert_gz_to_ros(gz_msg.torque(), ros_msg.wrench.torque);
}

}  // namespace ros_gz_bridge
