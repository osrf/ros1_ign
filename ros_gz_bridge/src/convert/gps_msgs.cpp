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

#include <cmath>

#include "convert/utils.hpp"
#include "ros_gz_bridge/convert/gps_msgs.hpp"

namespace ros_gz_bridge
{

template<>
void
convert_ros_to_gz(
  const gps_msgs::msg::GPSFix & ros_msg,
  gz::msgs::NavSat & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  gz_msg.set_latitude_deg(ros_msg.latitude);
  gz_msg.set_longitude_deg(ros_msg.longitude);
  gz_msg.set_altitude(ros_msg.altitude);
  gz_msg.set_frame_id(ros_msg.header.frame_id);

  gz_msg.set_velocity_east(cos(ros_msg.track) * ros_msg.speed);
  gz_msg.set_velocity_north(sin(ros_msg.track) * ros_msg.speed);
  gz_msg.set_velocity_up(ros_msg.climb);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::NavSat & gz_msg,
  gps_msgs::msg::GPSFix & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  ros_msg.header.frame_id = frame_id_gz_to_ros(gz_msg.frame_id());
  ros_msg.latitude = gz_msg.latitude_deg();
  ros_msg.longitude = gz_msg.longitude_deg();
  ros_msg.altitude = gz_msg.altitude();

  ros_msg.speed = sqrt(
    gz_msg.velocity_east() * gz_msg.velocity_east() +
    gz_msg.velocity_north() * gz_msg.velocity_north());
  ros_msg.track = atan2(gz_msg.velocity_north(), gz_msg.velocity_east());
  ros_msg.climb = gz_msg.velocity_up();

<<<<<<< HEAD
  // position_covariance is not supported in Ignition::Msgs::NavSat.
=======
  // position_covariance is not supported in gz::msgs::NavSat.
>>>>>>> 63b651a (Garden EOL (#662))
  ros_msg.position_covariance_type = gps_msgs::msg::GPSFix::COVARIANCE_TYPE_UNKNOWN;
  ros_msg.status.status = gps_msgs::msg::GPSStatus::STATUS_GBAS_FIX;
}

}  // namespace ros_gz_bridge
