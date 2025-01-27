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
#include <chrono>
#include <future>
#include <functional>
#include <memory>

#include <gz/math/Pose3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <rcpputils/scope_exit.hpp>
#include <std_msgs/msg/string.hpp>

// ROS interface for spawning entities into Gazebo.
// Suggested for use with roslaunch and loading entities from ROS param.
// If these are not needed, just use the `gz service` command line instead.

// \TODO(anyone) Remove GFlags in ROS-J
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

// Utility Function to avoid code duplication

bool set_XML_from_topic(
  const std::string & topic_name, const rclcpp::Node::SharedPtr ros2_node,
  gz::msgs::EntityFactory & req)
{
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
    topic_name, rclcpp::QoS(1).transient_local(), fun);

  rclcpp::FutureReturnCode future_ret;
  do {
    RCLCPP_INFO(ros2_node->get_logger(), "Waiting messages on topic [%s].", topic_name.c_str());
    future_ret = executor.spin_until_future_complete(xml_future, timeout);
  } while (rclcpp::ok() && future_ret != rclcpp::FutureReturnCode::SUCCESS);

  if (future_ret == rclcpp::FutureReturnCode::SUCCESS) {
    req.set_sdf(xml_future.get());
    return true;
  } else {
    RCLCPP_ERROR(
      ros2_node->get_logger(), "Failed to get XML from topic [%s].", topic_name.c_str());
    return false;
  }
}

int main(int _argc, char ** _argv)
{
  auto filtered_arguments = rclcpp::init_and_remove_ros_arguments(_argc, _argv);
  auto ros2_node = rclcpp::Node::make_shared("ros_gz_sim");

  // Construct a new argc/argv pair from the flags that weren't parsed by ROS
  // Gflags wants a mutable pointer to argv, which is why we can't use a
  // vector of strings here
  int filtered_argc = filtered_arguments.size();
  char ** filtered_argv = new char *[(filtered_argc + 1)];
  for (int ii = 0; ii < filtered_argc; ++ii) {
    filtered_argv[ii] = new char[filtered_arguments[ii].size() + 1];
    snprintf(
      filtered_argv[ii],
      filtered_arguments[ii].size() + 1, "%s", filtered_arguments[ii].c_str());
  }
  filtered_argv[filtered_argc] = nullptr;

  gflags::AllowCommandLineReparsing();
  gflags::SetUsageMessage(
    R"(Usage: create -world [arg] [-file FILE] [-param PARAM] [-topic TOPIC]
                       [-string STRING] [-name NAME] [-allow_renaming RENAMING] [-x X] [-y Y] [-z Z]
                       [-R ROLL] [-P PITCH] [-Y YAW])");
  gflags::ParseCommandLineFlags(&filtered_argc, &filtered_argv, false);

  // Free our temporary argc/argv pair
  for (size_t ii = 0; filtered_argv[ii] != nullptr; ++ii) {
    delete[] filtered_argv[ii];
  }
  delete[] filtered_argv;

  // Declare ROS 2 parameters to be passed from launch file
  ros2_node->declare_parameter("world", "");
  ros2_node->declare_parameter("file", "");
  ros2_node->declare_parameter("string", "");
  ros2_node->declare_parameter("topic", "");
  ros2_node->declare_parameter("name", "");
  ros2_node->declare_parameter("allow_renaming", false);
  ros2_node->declare_parameter("x", static_cast<double>(0));
  ros2_node->declare_parameter("y", static_cast<double>(0));
  ros2_node->declare_parameter("z", static_cast<double>(0));
  ros2_node->declare_parameter("R", static_cast<double>(0));
  ros2_node->declare_parameter("P", static_cast<double>(0));
  ros2_node->declare_parameter("Y", static_cast<double>(0));

  auto always_shutdown = rcpputils::make_scope_exit(
    []() {rclcpp::shutdown();});

  // World
  std::string world_name = ros2_node->get_parameter("world").as_string();
  if (world_name.empty() && !FLAGS_world.empty()) {
    world_name = FLAGS_world;
  }
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

  // Get ROS parameters
  std::string file_name = ros2_node->get_parameter("file").as_string();
  std::string xml_string = ros2_node->get_parameter("string").as_string();
  std::string topic_name = ros2_node->get_parameter("topic").as_string();
  // Check for the SDF filename or XML string or topic name
  if (!file_name.empty()) {
    req.set_sdf_filename(file_name);
  } else if (!xml_string.empty()) {
    req.set_sdf(xml_string);
  } else if (!topic_name.empty()) {
    // set XML string by fetching it from the given topic
    if (!set_XML_from_topic(topic_name, ros2_node, req)) {
      return -1;
    }
  } else if (filtered_arguments.size() > 1) {
    // Revert to Gflags, if ROS parameters aren't specified
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
      // set XML string by fetching it from the given topic
      if (!set_XML_from_topic(FLAGS_topic, ros2_node, req)) {
        return -1;
      }
    } else {
      RCLCPP_ERROR(
        ros2_node->get_logger(), "Must specify either -file, -param, -string or -topic");
      return -1;
    }
    // TODO(azeey) Deprecate use of command line flags in ROS 2 K-turtle in
    // favor of ROS 2 parameters.
  } else {
    RCLCPP_ERROR(
      ros2_node->get_logger(), "Must specify either file, string or topic as ROS 2 parameters");
    return -1;
  }

  // Pose
  double x_coords = ros2_node->get_parameter("x").as_double();
  double y_coords = ros2_node->get_parameter("y").as_double();
  double z_coords = ros2_node->get_parameter("z").as_double();
  double roll = ros2_node->get_parameter("R").as_double();
  double pitch = ros2_node->get_parameter("P").as_double();
  double yaw = ros2_node->get_parameter("Y").as_double();

  FLAGS_x = (x_coords != 0.0) ? x_coords : FLAGS_x;
  FLAGS_y = (y_coords != 0.0) ? y_coords : FLAGS_y;
  FLAGS_z = (z_coords != 0.0) ? z_coords : FLAGS_z;
  FLAGS_R = (roll != 0.0) ? roll : FLAGS_R;
  FLAGS_P = (pitch != 0.0) ? pitch : FLAGS_P;
  FLAGS_Y = (yaw != 0.0) ? yaw : FLAGS_Y;
  gz::math::Pose3d pose{FLAGS_x, FLAGS_y, FLAGS_z, FLAGS_R, FLAGS_P, FLAGS_Y};
  gz::msgs::Set(req.mutable_pose(), pose);

  // Name
  std::string entity_name = ros2_node->get_parameter("name").as_string();
  if (!entity_name.empty()) {
    req.set_name(entity_name);
  } else {
    req.set_name(FLAGS_name);
  }

  // Allow Renaming
  bool allow_renaming = ros2_node->get_parameter("allow_renaming").as_bool();
  req.set_allow_renaming((allow_renaming || FLAGS_allow_renaming));

  // Request
  gz::transport::Node node;
  gz::msgs::Boolean rep;
  bool result;
  unsigned int timeout = 5000;

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
    } else {
      RCLCPP_WARN(
        ros2_node->get_logger(), "Waiting for service [%s] to become available ...",
        service.c_str());
    }
  }
  RCLCPP_INFO(ros2_node->get_logger(), "Entity creation was interrupted.");
  return 0;
}
