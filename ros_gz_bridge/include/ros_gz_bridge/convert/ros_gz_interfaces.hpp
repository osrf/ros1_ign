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

#ifndef ROS_GZ_BRIDGE__CONVERT__ROS_GZ_INTERFACES_HPP_
#define ROS_GZ_BRIDGE__CONVERT__ROS_GZ_INTERFACES_HPP_

// Gazebo Msgs
#include <gz/msgs/altimeter.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_wrench.pb.h>
#include <gz/msgs/joint_wrench.pb.h>
#include <gz/msgs/contact.pb.h>
#include <gz/msgs/contacts.pb.h>
<<<<<<< HEAD
#include <gz/msgs/float_v.pb.h>
#include <gz/msgs/gui_camera.pb.h>
#include <gz/msgs/light.pb.h>
=======
#include <gz/msgs/dataframe.pb.h>
#include <gz/msgs/float_v.pb.h>
#include <gz/msgs/gui_camera.pb.h>
#include <gz/msgs/light.pb.h>
#include <gz/msgs/material_color.pb.h>
>>>>>>> 63b651a (Garden EOL (#662))
#include <gz/msgs/param.pb.h>
#include <gz/msgs/param_v.pb.h>
#include <gz/msgs/sensor_noise.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>
#include <gz/msgs/track_visual.pb.h>
#include <gz/msgs/video_record.pb.h>
#include <gz/msgs/world_control.pb.h>

// ROS 2 messages
#include <ros_gz_interfaces/msg/altimeter.hpp>
#include <ros_gz_interfaces/msg/entity.hpp>
#include <ros_gz_interfaces/msg/entity_wrench.hpp>
#include <ros_gz_interfaces/msg/joint_wrench.hpp>
#include <ros_gz_interfaces/msg/contact.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
<<<<<<< HEAD
#include <ros_gz_interfaces/msg/float32_array.hpp>
#include <ros_gz_interfaces/msg/gui_camera.hpp>
#include <ros_gz_interfaces/msg/light.hpp>
=======
#include <ros_gz_interfaces/msg/dataframe.hpp>
#include <ros_gz_interfaces/msg/float32_array.hpp>
#include <ros_gz_interfaces/msg/gui_camera.hpp>
#include <ros_gz_interfaces/msg/light.hpp>
#include <ros_gz_interfaces/msg/material_color.hpp>
>>>>>>> 63b651a (Garden EOL (#662))
#include <ros_gz_interfaces/msg/param_vec.hpp>
#include <ros_gz_interfaces/msg/sensor_noise.hpp>
#include <ros_gz_interfaces/msg/string_vec.hpp>
#include <ros_gz_interfaces/msg/track_visual.hpp>
#include <ros_gz_interfaces/msg/video_record.hpp>
#include <ros_gz_interfaces/msg/world_control.hpp>

<<<<<<< HEAD
// Required for HAVE_DATAFRAME definition
#include <ros_gz_bridge/ros_gz_bridge.hpp>

#if HAVE_DATAFRAME
#include <gz/msgs/dataframe.pb.h>
#include <ros_gz_interfaces/msg/dataframe.hpp>
#endif  // HAVE_DATAFRAME

#if HAVE_MATERIALCOLOR
#include <gz/msgs/material_color.pb.h>
#include <ros_gz_interfaces/msg/material_color.hpp>
#endif  // HAVE_MATERIALCOLOR

=======
#include <ros_gz_bridge/ros_gz_bridge.hpp>

>>>>>>> 63b651a (Garden EOL (#662))
#include <ros_gz_bridge/convert_decl.hpp>

namespace ros_gz_bridge
{

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::JointWrench & ros_msg,
  gz::msgs::JointWrench & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::JointWrench & gz_msg,
  ros_gz_interfaces::msg::JointWrench & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Altimeter & ros_msg,
  gz::msgs::Altimeter & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Altimeter & gz_msg,
  ros_gz_interfaces::msg::Altimeter & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Entity & ros_msg,
  gz::msgs::Entity & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Entity & gz_msg,
  ros_gz_interfaces::msg::Entity & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::EntityWrench & ros_msg,
  gz::msgs::EntityWrench & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::EntityWrench & gz_msg,
  ros_gz_interfaces::msg::EntityWrench & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Contact & ros_msg,
  gz::msgs::Contact & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Contact & gz_msg,
  ros_gz_interfaces::msg::Contact & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Contacts & ros_msg,
  gz::msgs::Contacts & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Contacts & gz_msg,
  ros_gz_interfaces::msg::Contacts & ros_msg);

<<<<<<< HEAD
#if HAVE_DATAFRAME
=======
>>>>>>> 63b651a (Garden EOL (#662))
template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Dataframe & ros_msg,
  gz::msgs::Dataframe & ign_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Dataframe & ign_msg,
  ros_gz_interfaces::msg::Dataframe & ros_msg);
<<<<<<< HEAD
#endif  // HAVE_DATAFRAME
=======
>>>>>>> 63b651a (Garden EOL (#662))

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::GuiCamera & ros_msg,
  gz::msgs::GUICamera & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::GUICamera & gz_msg,
  ros_gz_interfaces::msg::GuiCamera & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Light & ros_msg,
  gz::msgs::Light & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Light & gz_msg,
  ros_gz_interfaces::msg::Light & ros_msg);

<<<<<<< HEAD
#if HAVE_MATERIALCOLOR
=======
>>>>>>> 63b651a (Garden EOL (#662))
template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::MaterialColor & ros_msg,
  gz::msgs::MaterialColor & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::MaterialColor & gz_msg,
  ros_gz_interfaces::msg::MaterialColor & ros_msg);
<<<<<<< HEAD
#endif  // HAVE_MATERIALCOLOR
=======
>>>>>>> 63b651a (Garden EOL (#662))

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::SensorNoise & ros_msg,
  gz::msgs::SensorNoise & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::SensorNoise & gz_msg,
  ros_gz_interfaces::msg::SensorNoise & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::StringVec & ros_msg,
  gz::msgs::StringMsg_V & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::StringMsg_V & gz_msg,
  ros_gz_interfaces::msg::StringVec & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::ParamVec & ros_msg,
  gz::msgs::Param & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Param & gz_msg,
  ros_gz_interfaces::msg::ParamVec & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::ParamVec & ros_msg,
  gz::msgs::Param_V & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Param_V & gz_msg,
  ros_gz_interfaces::msg::ParamVec & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::TrackVisual & ros_msg,
  gz::msgs::TrackVisual & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::TrackVisual & gz_msg,
  ros_gz_interfaces::msg::TrackVisual & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::VideoRecord & ros_msg,
  gz::msgs::VideoRecord & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::VideoRecord & gz_msg,
  ros_gz_interfaces::msg::VideoRecord & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::WorldControl & ros_msg,
  gz::msgs::WorldControl & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::WorldControl & gz_msg,
  ros_gz_interfaces::msg::WorldControl & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::WorldReset & ros_msg,
  gz::msgs::WorldReset & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::WorldReset & gz_msg,
  ros_gz_interfaces::msg::WorldReset & ros_msg);

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Float32Array & ros_msg,
  gz::msgs::Float_V & gz_msg);

template<>
void
convert_gz_to_ros(
  const gz::msgs::Float_V & gz_msg,
  ros_gz_interfaces::msg::Float32Array & ros_msg);
}  // namespace ros_gz_bridge

#endif  // ROS_GZ_BRIDGE__CONVERT__ROS_GZ_INTERFACES_HPP_
