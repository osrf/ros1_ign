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

<<<<<<< HEAD
=======
#include <yaml-cpp/exceptions.h>
>>>>>>> 63b651a (Garden EOL (#662))
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/bridge_config.hpp>

namespace ros_gz_bridge
{

// YAML tag string constants
constexpr const char kTopicName[] = "topic_name";
constexpr const char kRosTopicName[] = "ros_topic_name";
constexpr const char kGzTopicName[] = "gz_topic_name";
constexpr const char kRosTypeName[] = "ros_type_name";
constexpr const char kGzTypeName[] = "gz_type_name";
constexpr const char kDirection[] = "direction";
constexpr const char kPublisherQueue[] = "publisher_queue";
constexpr const char kSubscriberQueue[] = "subscriber_queue";
constexpr const char kLazy[] = "lazy";

// Comparison strings for bridge directions
constexpr const char kBidirectional[] = "BIDIRECTIONAL";
constexpr const char kGzToRos[] = "GZ_TO_ROS";
constexpr const char kRosToGz[] = "ROS_TO_GZ";

<<<<<<< HEAD
/// \TODO(mjcarroll) Remove these in releases past Humble/Garden
constexpr const char kIgnTypeName[] = "ign_type_name";
constexpr const char kIgnTopicName[] = "ign_topic_name";
constexpr const char kIgnToRos[] = "IGN_TO_ROS";
constexpr const char kRosToIgn[] = "ROS_TO_IGN";

=======
>>>>>>> 63b651a (Garden EOL (#662))
/// \brief Parse a single sequence entry into a BridgeConfig
/// \param[in] yaml_node A node containing a map of bridge config params
/// \return BridgeConfig on success, nullopt on failure
std::optional<BridgeConfig> parseEntry(const YAML::Node & yaml_node)
{
  auto logger = rclcpp::get_logger("BridgeConfig");

  if (!yaml_node.IsMap()) {
    RCLCPP_ERROR(
      logger,
      "Could not parse entry: entry must be a YAML map");
    return {};
  }

<<<<<<< HEAD
  /// \TODO(mjcarroll) Remove gz_type_name logic in releases past Humble
  std::string gz_type_name = "";
  if (yaml_node[kIgnTypeName] && !yaml_node[kGzTypeName]) {
    gz_type_name = yaml_node[kIgnTypeName].as<std::string>();
    RCLCPP_ERROR(
      logger,
      "%s is deprecated, migrate to %s", kIgnTypeName, kGzTypeName);
  } else if (yaml_node[kGzTypeName]) {
    gz_type_name = yaml_node[kGzTypeName].as<std::string>();
  }

  /// \TODO(mjcarroll) Remove gz_topic_name logic in releases past Humble
  std::string gz_topic_name = "";
  if (yaml_node[kIgnTopicName] && !yaml_node[kGzTopicName]) {
    gz_topic_name = yaml_node[kIgnTopicName].as<std::string>();
    RCLCPP_ERROR(
      logger,
      "%s is deprecated, migrate to %s", kIgnTopicName, kGzTopicName);
  } else if (yaml_node[kGzTopicName]) {
    gz_topic_name = yaml_node[kGzTopicName].as<std::string>();
  }

  if (yaml_node[kTopicName] && yaml_node[kRosTopicName]) {
=======
  auto getValue = [yaml_node](const char * key) -> std::string
    {
      if (yaml_node[key]) {
        return yaml_node[key].as<std::string>();
      } else {
        return "";
      }
    };

  const auto topic_name = getValue(kTopicName);
  const auto ros_topic_name = getValue(kRosTopicName);
  const auto ros_type_name = getValue(kRosTypeName);
  const auto gz_topic_name = getValue(kGzTopicName);
  const auto gz_type_name = getValue(kGzTypeName);
  const auto direction = getValue(kDirection);

  if (!topic_name.empty() && !ros_topic_name.empty()) {
>>>>>>> 63b651a (Garden EOL (#662))
    RCLCPP_ERROR(
      logger,
      "Could not parse entry: %s and %s are mutually exclusive", kTopicName, kRosTopicName);
    return {};
  }

<<<<<<< HEAD
  if (yaml_node[kTopicName] && !gz_topic_name.empty()) {
=======
  if (!topic_name.empty() && !gz_topic_name.empty()) {
>>>>>>> 63b651a (Garden EOL (#662))
    RCLCPP_ERROR(
      logger,
      "Could not parse entry: %s and %s are mutually exclusive", kTopicName, kGzTopicName);
    return {};
  }

<<<<<<< HEAD
  if (!yaml_node[kRosTypeName] || gz_type_name.empty()) {
=======
  if (ros_type_name.empty() || gz_type_name.empty()) {
>>>>>>> 63b651a (Garden EOL (#662))
    RCLCPP_ERROR(
      logger,
      "Could not parse entry: both %s and %s must be set", kRosTypeName, kGzTypeName);
    return {};
  }

  BridgeConfig ret;

  ret.direction = BridgeDirection::BIDIRECTIONAL;

  if (yaml_node[kDirection]) {
<<<<<<< HEAD
    auto dirStr = yaml_node[kDirection].as<std::string>();

    if (dirStr == kBidirectional) {
      ret.direction = BridgeDirection::BIDIRECTIONAL;
    } else if (dirStr == kGzToRos) {
      ret.direction = BridgeDirection::GZ_TO_ROS;
    } else if (dirStr == kRosToGz) {
      ret.direction = BridgeDirection::ROS_TO_GZ;
    } else if (dirStr == kIgnToRos) {
      ret.direction = BridgeDirection::GZ_TO_ROS;
      RCLCPP_WARN(
        logger,
        "%s constant is deprecated, migrate to %s", kIgnToRos, kGzToRos);
    } else if (dirStr == kRosToIgn) {
      ret.direction = BridgeDirection::ROS_TO_GZ;
      RCLCPP_WARN(
        logger,
        "%s constant is deprecated, migrate to %s", kRosToIgn, kRosToGz);
    } else {
      RCLCPP_ERROR(
        logger,
        "Could not parse entry: invalid direction [%s]", dirStr.c_str());
=======
    if (direction == kBidirectional) {
      ret.direction = BridgeDirection::BIDIRECTIONAL;
    } else if (direction == kGzToRos) {
      ret.direction = BridgeDirection::GZ_TO_ROS;
    } else if (direction == kRosToGz) {
      ret.direction = BridgeDirection::ROS_TO_GZ;
    } else {
      RCLCPP_ERROR(
        logger,
        "Could not parse entry: invalid direction [%s]", direction.c_str());
>>>>>>> 63b651a (Garden EOL (#662))
      return {};
    }
  }

<<<<<<< HEAD
  if (yaml_node[kTopicName]) {
    // Only "topic_name" is set
    ret.gz_topic_name = yaml_node[kTopicName].as<std::string>();
    ret.ros_topic_name = yaml_node[kTopicName].as<std::string>();
  } else if (yaml_node[kRosTopicName] && gz_topic_name.empty()) {
    // Only "ros_topic_name" is set
    ret.gz_topic_name = yaml_node[kRosTopicName].as<std::string>();
    ret.ros_topic_name = yaml_node[kRosTopicName].as<std::string>();
  } else if (!gz_topic_name.empty() && !yaml_node[kRosTopicName]) {
=======
  if (!topic_name.empty()) {
    // Only "topic_name" is set
    ret.gz_topic_name = topic_name;
    ret.ros_topic_name = topic_name;
  } else if (!ros_topic_name.empty() && gz_topic_name.empty()) {
    // Only "ros_topic_name" is set
    ret.gz_topic_name = ros_topic_name;
    ret.ros_topic_name = ros_topic_name;
  } else if (!gz_topic_name.empty() && ros_topic_name.empty()) {
>>>>>>> 63b651a (Garden EOL (#662))
    // Only kGzTopicName is set
    ret.gz_topic_name = gz_topic_name;
    ret.ros_topic_name = gz_topic_name;
  } else {
    // Both are set
    ret.gz_topic_name = gz_topic_name;
<<<<<<< HEAD
    ret.ros_topic_name = yaml_node[kRosTopicName].as<std::string>();
  }

  ret.gz_type_name = gz_type_name;
  ret.ros_type_name = yaml_node[kRosTypeName].as<std::string>();
=======
    ret.ros_topic_name = ros_topic_name;
  }

  ret.gz_type_name = gz_type_name;
  ret.ros_type_name = ros_type_name;
>>>>>>> 63b651a (Garden EOL (#662))

  if (yaml_node[kPublisherQueue]) {
    ret.publisher_queue_size = yaml_node[kPublisherQueue].as<size_t>();
  }
  if (yaml_node[kSubscriberQueue]) {
    ret.subscriber_queue_size = yaml_node[kSubscriberQueue].as<size_t>();
  }
  if (yaml_node[kLazy]) {
    ret.is_lazy = yaml_node[kLazy].as<bool>();
  }

  return ret;
}

std::vector<BridgeConfig> readFromYaml(std::istream & in)
{
<<<<<<< HEAD
=======
  auto logger = rclcpp::get_logger("readFromYaml");
>>>>>>> 63b651a (Garden EOL (#662))
  auto ret = std::vector<BridgeConfig>();

  YAML::Node yaml_node;
  yaml_node = YAML::Load(in);

<<<<<<< HEAD
  auto logger = rclcpp::get_logger("readFromYaml");
  if (!yaml_node.IsSequence()) {
    RCLCPP_ERROR(
      logger,
      "Could not parse config, top level must be a YAML sequence");
=======
  if (!yaml_node.IsSequence()) {
    RCLCPP_ERROR(
      logger,
      "Could not parse config: top level must be a YAML sequence");
>>>>>>> 63b651a (Garden EOL (#662))
    return ret;
  }

  for (auto it : yaml_node) {
    auto entry = parseEntry(it);
    if (entry) {
      ret.push_back(entry.value());
    }
  }

  return ret;
}

std::vector<BridgeConfig> readFromYamlFile(const std::string & filename)
{
  std::vector<BridgeConfig> ret;
  std::ifstream in(filename);
<<<<<<< HEAD
=======

  auto logger = rclcpp::get_logger("readFromYamlFile");
  if (!in.is_open()) {
    RCLCPP_ERROR(
      logger,
      "Could not parse config: failed to open file [%s]", filename.c_str());
    return ret;
  }

  // Compute file size to warn on empty configuration
  const auto fbegin = in.tellg();
  in.seekg(0, std::ios::end);
  const auto fend = in.tellg();
  const auto fsize = fend - fbegin;

  if (fsize == 0) {
    RCLCPP_ERROR(
      logger,
      "Could not parse config: file empty [%s]", filename.c_str());
    return ret;
  }

  in.seekg(0, std::ios::beg);
>>>>>>> 63b651a (Garden EOL (#662))
  return readFromYaml(in);
}

std::vector<BridgeConfig> readFromYamlString(const std::string & data)
{
  std::stringstream ss(data);
  return readFromYaml(ss);
}
}  // namespace ros_gz_bridge
