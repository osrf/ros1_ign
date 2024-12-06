// Copyright 2023 Rudis Laboratories LLC
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
#include "ros_gz_bridge/convert/actuator_msgs.hpp"

namespace ros_gz_bridge
{
<<<<<<< HEAD

=======
>>>>>>> 63b651a (Garden EOL (#662))
template<>
void
convert_ros_to_gz(
  const actuator_msgs::msg::Actuators & ros_msg,
  gz::msgs::Actuators & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));

  for (auto i = 0u; i < ros_msg.position.size(); ++i) {
    gz_msg.add_position(ros_msg.position[i]);
  }

  for (auto i = 0u; i < ros_msg.velocity.size(); ++i) {
    gz_msg.add_velocity(ros_msg.velocity[i]);
  }
  for (auto i = 0u; i < ros_msg.normalized.size(); ++i) {
    gz_msg.add_normalized(ros_msg.normalized[i]);
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Actuators & gz_msg,
  actuator_msgs::msg::Actuators & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  for (auto i = 0; i < gz_msg.position_size(); ++i) {
    ros_msg.position.push_back(gz_msg.position(i));
  }

  for (auto i = 0; i < gz_msg.velocity_size(); ++i) {
    ros_msg.velocity.push_back(gz_msg.velocity(i));
  }

  for (auto i = 0; i < gz_msg.normalized_size(); ++i) {
    ros_msg.normalized.push_back(gz_msg.normalized(i));
  }
}

}  // namespace ros_gz_bridge
