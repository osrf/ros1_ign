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

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>

#include <memory>
#include <string>

#include "service_factories/ros_gz_interfaces.hpp"
#include "ros_gz_interfaces/srv/control_world.hpp"
#include "ros_gz_bridge/convert/ros_gz_interfaces.hpp"

#include "service_factory.hpp"

namespace ros_gz_bridge
{

std::shared_ptr<ServiceFactoryInterface>
get_service_factory__ros_gz_interfaces(
  const std::string & ros_type_name,
  const std::string & gz_req_type_name,
  const std::string & gz_rep_type_name)
{
  if (
    ros_type_name == "ros_gz_interfaces/srv/ControlWorld" &&
<<<<<<< HEAD
    (gz_req_type_name.empty() || gz_req_type_name == "ignition.msgs.WorldControl") &&
    (gz_rep_type_name.empty() || gz_rep_type_name == "ignition.msgs.Boolean"))
=======
    (gz_req_type_name.empty() || gz_req_type_name == "gz.msgs.WorldControl") &&
    (gz_rep_type_name.empty() || gz_rep_type_name == "gz.msgs.Boolean"))
>>>>>>> 63b651a (Garden EOL (#662))
  {
    return std::make_shared<
      ServiceFactory<
        ros_gz_interfaces::srv::ControlWorld,
        gz::msgs::WorldControl,
        gz::msgs::Boolean>
<<<<<<< HEAD
    >(ros_type_name, "ignition.msgs.WorldControl", "ignition.msgs.Boolean");
=======
    >(ros_type_name, "gz.msgs.WorldControl", "gz.msgs.Boolean");
>>>>>>> 63b651a (Garden EOL (#662))
  }

  return nullptr;
}

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::srv::ControlWorld::Request & ros_req,
  gz::msgs::WorldControl & gz_req)
{
  convert_ros_to_gz(ros_req.world_control, gz_req);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Boolean & gz_rep,
  ros_gz_interfaces::srv::ControlWorld::Response & ros_res)
{
  ros_res.success = gz_rep.data();
}

template<>
bool
send_response_on_error(ros_gz_interfaces::srv::ControlWorld::Response & ros_res)
{
  // TODO(now): Is it worth it to have a different field to encode Gazebo request errors?
  //  Currently we're reusing the success field, which seems fine for this case.
  ros_res.success = false;
  return true;
}
}  // namespace ros_gz_bridge
