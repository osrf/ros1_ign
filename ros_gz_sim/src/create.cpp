// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <gflags/gflags.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

#include <sstream>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


DEFINE_string(world, "", "World name.");
DEFINE_string(file, "", "Load XML from a file.");
DEFINE_string(param, "", "Load XML from a ROS param.");
DEFINE_string(string, "", "Load XML from a string.");
DEFINE_string(topic, "", "Load XML from a ROS string publisher.");
DEFINE_string(name, "", "Name for spawned entity.");
DEFINE_bool(allow_renaming, false, "Rename entity if name already used.");
DEFINE_double(x, 0, "X component of initial position, in meters.");
DEFINE_double(y, 0, "Y component of initial position, in meters.");
DEFINE_double(z, 0, "Z component of initial position, in meters.");
DEFINE_double(R, 0, "Roll component of initial orientation, in radians.");
DEFINE_double(P, 0, "Pitch component of initial orientation, in radians.");
DEFINE_double(Y, 0, "Yaw component of initial orientation, in radians.");

// ROS interface for spawning entities into Gazebo.
// Suggested for use with roslaunch and loading entities from ROS param.
// If these are not needed, just use the `gz service` command line instead.
int main(int _argc, char ** _argv)
{
  rclcpp::init(_argc, _argv);
  auto ros2_node = rclcpp::Node::make_shared("ros_gz_sim");

  gflags::AllowCommandLineReparsing();
  gflags::SetUsageMessage(
    R"(Usage: create -world [arg] [-file FILE] [-param PARAM] [-topic TOPIC]
                       [-string STRING] [-name NAME] [-allow_renaming RENAMING] [-x X] [-y Y] [-z Z]
                       [-R ROLL] [-P PITCH] [-Y YAW])");
  gflags::ParseCommandLineFlags(&_argc, &_argv, true);

  // World
  std::string world_name = FLAGS_world;
  if (world_name.empty()) {
    // If caller doesn't provide a world name, get list of worlds from gz-sim server
    gz::transport::Node node;

    bool executed{false};
    bool result{false};
    unsigned int timeout{5000};
    std::string service{"/gazebo/worlds"};
    gz::msgs::StringMsg_V worlds_msg;

    // This loop is here to allow the server time to download resources.
    while (rclcpp::ok() && !executed) {
      RCLCPP_INFO(ros2_node->get_logger(), "Requesting list of world names.");
      executed = node.Request(service, timeout, worlds_msg, result);
    }

    if (!executed) {
      RCLCPP_INFO(ros2_node->get_logger(), "Timed out when getting world names.");
      return -1;
    }

    if (!result || worlds_msg.data().empty()) {
      RCLCPP_INFO(ros2_node->get_logger(), "Failed to get world names.");
      return -1;
    }

    world_name = worlds_msg.data(0);
  }
  std::string service{"/world/" + world_name + "/create"};

  // Request message
  gz::msgs::EntityFactory req;

  // File
  if (!FLAGS_file.empty()) {
    req.set_sdf_filename(FLAGS_file);
  } else if (!FLAGS_param.empty()) {  // Param
    ros2_node->declare_parameter<std::string>(FLAGS_param);

    std::string xmlStr;
    if (ros2_node->get_parameter(FLAGS_param, xmlStr)) {
      req.set_sdf(xmlStr);
    } else {
      RCLCPP_ERROR(
        ros2_node->get_logger(), "Failed to get XML from param [%s].", FLAGS_param.c_str());
      return -1;
    }
  } else if (!FLAGS_string.empty()) {  // string
    req.set_sdf(FLAGS_string);
  } else if (!FLAGS_topic.empty()) {  // topic
    const auto timeout = std::chrono::seconds(1);
    std::promise<std::string> xml_promise;
    std::shared_future<std::string> xml_future(xml_promise.get_future());

    std::function<void(const std_msgs::msg::String::SharedPtr)> fun =
      [&xml_promise](const std_msgs::msg::String::SharedPtr msg) {
        xml_promise.set_value(msg->data);
      };

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(ros2_node);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_subs;
    // Transient local is similar to latching in ROS 1.
    description_subs = ros2_node->create_subscription<std_msgs::msg::String>(
      FLAGS_topic, rclcpp::QoS(1).transient_local(), fun);

    rclcpp::FutureReturnCode future_ret;
    do {
      RCLCPP_INFO(ros2_node->get_logger(), "Waiting messages on topic [%s].", FLAGS_topic.c_str());
      future_ret = executor.spin_until_future_complete(xml_future, timeout);
    } while (rclcpp::ok() && future_ret != rclcpp::FutureReturnCode::SUCCESS);

    if (future_ret == rclcpp::FutureReturnCode::SUCCESS) {
      req.set_sdf(xml_future.get());
    } else {
      RCLCPP_ERROR(
        ros2_node->get_logger(), "Failed to get XML from topic [%s].", FLAGS_topic.c_str());
      return -1;
    }
  } else {
    RCLCPP_ERROR(ros2_node->get_logger(), "Must specify either -file, -param, -stdin or -topic");
    return -1;
  }

  // Pose
  gz::math::Pose3d pose{FLAGS_x, FLAGS_y, FLAGS_z, FLAGS_R, FLAGS_P, FLAGS_Y};
  gz::msgs::Set(req.mutable_pose(), pose);

  // Name
  if (!FLAGS_name.empty()) {
    req.set_name(FLAGS_name);
  }

  if (FLAGS_allow_renaming) {
    req.set_allow_renaming(FLAGS_allow_renaming);
  }

  // Request
  gz::transport::Node node;
  gz::msgs::Boolean rep;
  bool result;
  unsigned int timeout = 5000;
  bool executed = node.Request(service, req, timeout, rep, result);

<<<<<<< HEAD
  if (executed) {
    if (result && rep.data()) {
      RCLCPP_INFO(ros2_node->get_logger(), "Requested creation of entity.");
=======
  while(rclcpp::ok()) {
    if (node.Request(service, req, timeout, rep, result)) {
      if (result && rep.data()) {
        RCLCPP_INFO(ros2_node->get_logger(), "Entity creation successful.");
        return 0;
      } else {
        RCLCPP_ERROR(
          ros2_node->get_logger(), "Entity creation failed.\n %s",
          req.DebugString().c_str());
        return 1;
      }
>>>>>>> 5e3b073 (Fix spelling in entity creation (#688))
    } else {
      RCLCPP_ERROR(
        ros2_node->get_logger(), "Failed request to create entity.\n %s",
        req.DebugString().c_str());
    }
  } else {
    RCLCPP_ERROR(
      ros2_node->get_logger(), "Request to create entity from service [%s] timed out..\n %s",
      service.c_str(), req.DebugString().c_str());
  }
  RCLCPP_INFO(ros2_node->get_logger(), "OK creation of entity.");

  return 0;
}
