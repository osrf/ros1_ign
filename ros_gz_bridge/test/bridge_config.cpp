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

#include <gtest/gtest.h>

#include <ros_gz_bridge/bridge_config.hpp>

<<<<<<< HEAD
TEST(BridgeConfig, Minimum)
=======
#include "rcutils/logging.h"

size_t g_log_calls = 0;

struct LogEvent
{
  const rcutils_log_location_t * location;
  int level;
  std::string name;
  rcutils_time_point_value_t timestamp;
  std::string message;
};
LogEvent g_last_log_event;

class BridgeConfig : public ::testing::Test
{
public:
  rcutils_logging_output_handler_t previous_output_handler;
  void SetUp()
  {
    g_log_calls = 0;
    ASSERT_EQ(RCUTILS_RET_OK, rcutils_logging_initialize());
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);

    auto rcutils_logging_console_output_handler = [](
      const rcutils_log_location_t * location,
      int level, const char * name, rcutils_time_point_value_t timestamp,
      const char * format, va_list * args) -> void
      {
        g_log_calls += 1;
        g_last_log_event.location = location;
        g_last_log_event.level = level;
        g_last_log_event.name = name ? name : "";
        g_last_log_event.timestamp = timestamp;
        char buffer[1024];
        vsnprintf(buffer, sizeof(buffer), format, *args);
        g_last_log_event.message = buffer;
      };

    this->previous_output_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(rcutils_logging_console_output_handler);
  }

  void TearDown()
  {
    rcutils_logging_set_output_handler(this->previous_output_handler);
    ASSERT_EQ(RCUTILS_RET_OK, rcutils_logging_shutdown());
    EXPECT_FALSE(g_rcutils_logging_initialized);
  }
};


TEST_F(BridgeConfig, Minimum)
>>>>>>> 63b651a (Garden EOL (#662))
{
  auto results = ros_gz_bridge::readFromYamlFile("test/config/minimum.yaml");
  EXPECT_EQ(4u, results.size());

  {
    auto config = results[0];
    EXPECT_EQ("chatter", config.ros_topic_name);
    EXPECT_EQ("chatter", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(ros_gz_bridge::kDefaultPublisherQueue, config.publisher_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultSubscriberQueue, config.subscriber_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultLazy, config.is_lazy);
  }
  {
    auto config = results[1];
    EXPECT_EQ("chatter_ros", config.ros_topic_name);
    EXPECT_EQ("chatter_ros", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(ros_gz_bridge::kDefaultPublisherQueue, config.publisher_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultSubscriberQueue, config.subscriber_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultLazy, config.is_lazy);
  }
  {
    auto config = results[2];
    EXPECT_EQ("chatter_gz", config.ros_topic_name);
    EXPECT_EQ("chatter_gz", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(ros_gz_bridge::kDefaultPublisherQueue, config.publisher_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultSubscriberQueue, config.subscriber_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultLazy, config.is_lazy);
  }
  {
    auto config = results[3];
    EXPECT_EQ("chatter_both_ros", config.ros_topic_name);
    EXPECT_EQ("chatter_both_gz", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(ros_gz_bridge::kDefaultPublisherQueue, config.publisher_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultSubscriberQueue, config.subscriber_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultLazy, config.is_lazy);
  }
}

<<<<<<< HEAD
TEST(BridgeConfig, MinimumIgn)
{
  auto results = ros_gz_bridge::readFromYamlFile("test/config/minimum_ign.yaml");
  EXPECT_EQ(4u, results.size());

  {
    auto config = results[0];
    EXPECT_EQ("chatter", config.ros_topic_name);
    EXPECT_EQ("chatter", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(ros_gz_bridge::kDefaultPublisherQueue, config.publisher_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultSubscriberQueue, config.subscriber_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultLazy, config.is_lazy);
  }
  {
    auto config = results[1];
    EXPECT_EQ("chatter_ros", config.ros_topic_name);
    EXPECT_EQ("chatter_ros", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(ros_gz_bridge::kDefaultPublisherQueue, config.publisher_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultSubscriberQueue, config.subscriber_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultLazy, config.is_lazy);
  }
  {
    auto config = results[2];
    EXPECT_EQ("chatter_gz", config.ros_topic_name);
    EXPECT_EQ("chatter_gz", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(ros_gz_bridge::kDefaultPublisherQueue, config.publisher_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultSubscriberQueue, config.subscriber_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultLazy, config.is_lazy);
  }
  {
    auto config = results[3];
    EXPECT_EQ("chatter_both_ros", config.ros_topic_name);
    EXPECT_EQ("chatter_both_gz", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(ros_gz_bridge::kDefaultPublisherQueue, config.publisher_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultSubscriberQueue, config.subscriber_queue_size);
    EXPECT_EQ(ros_gz_bridge::kDefaultLazy, config.is_lazy);
  }
}


TEST(BridgeConfig, FullGz)
=======
TEST_F(BridgeConfig, FullGz)
>>>>>>> 63b651a (Garden EOL (#662))
{
  auto results = ros_gz_bridge::readFromYamlFile("test/config/full.yaml");
  EXPECT_EQ(2u, results.size());

  {
    auto config = results[0];
    EXPECT_EQ("ros_chatter", config.ros_topic_name);
    EXPECT_EQ("gz_chatter", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(6u, config.publisher_queue_size);
    EXPECT_EQ(5u, config.subscriber_queue_size);
    EXPECT_EQ(true, config.is_lazy);
    EXPECT_EQ(ros_gz_bridge::BridgeDirection::ROS_TO_GZ, config.direction);
  }

  {
    auto config = results[1];
    EXPECT_EQ("ros_chatter", config.ros_topic_name);
    EXPECT_EQ("gz_chatter", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(20u, config.publisher_queue_size);
    EXPECT_EQ(10u, config.subscriber_queue_size);
    EXPECT_EQ(false, config.is_lazy);
    EXPECT_EQ(ros_gz_bridge::BridgeDirection::GZ_TO_ROS, config.direction);
  }
}

<<<<<<< HEAD
TEST(BridgeConfig, FullIgn)
{
  auto results = ros_gz_bridge::readFromYamlFile("test/config/full.yaml");
  EXPECT_EQ(2u, results.size());

  {
    auto config = results[0];
    EXPECT_EQ("ros_chatter", config.ros_topic_name);
    EXPECT_EQ("gz_chatter", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(6u, config.publisher_queue_size);
    EXPECT_EQ(5u, config.subscriber_queue_size);
    EXPECT_EQ(true, config.is_lazy);
    EXPECT_EQ(ros_gz_bridge::BridgeDirection::ROS_TO_GZ, config.direction);
  }

  {
    auto config = results[1];
    EXPECT_EQ("ros_chatter", config.ros_topic_name);
    EXPECT_EQ("gz_chatter", config.gz_topic_name);
    EXPECT_EQ("std_msgs/msg/String", config.ros_type_name);
    EXPECT_EQ("ignition.msgs.StringMsg", config.gz_type_name);
    EXPECT_EQ(20u, config.publisher_queue_size);
    EXPECT_EQ(10u, config.subscriber_queue_size);
    EXPECT_EQ(false, config.is_lazy);
    EXPECT_EQ(ros_gz_bridge::BridgeDirection::GZ_TO_ROS, config.direction);
  }
}

TEST(BridgeConfig, InvalidSetTwoRos)
=======
TEST_F(BridgeConfig, InvalidSetTwoRos)
>>>>>>> 63b651a (Garden EOL (#662))
{
  // Cannot set topic_name and ros_topic_name
  auto yaml = R"(
- topic_name: foo
  ros_topic_name: bar)";

  auto results = ros_gz_bridge::readFromYamlString(yaml);
  EXPECT_EQ(0u, results.size());
<<<<<<< HEAD
}

TEST(BridgeConfig, InvalidSetTwoGz)
=======
  EXPECT_EQ(
    "Could not parse entry: topic_name and ros_topic_name are mutually exclusive",
    g_last_log_event.message);
}

TEST_F(BridgeConfig, InvalidSetTwoGz)
>>>>>>> 63b651a (Garden EOL (#662))
{
  // Cannot set topic_name and gz_topic_name
  auto yaml = R"(
- topic_name: foo
  gz_topic_name: bar)";

  auto results = ros_gz_bridge::readFromYamlString(yaml);
  EXPECT_EQ(0u, results.size());
<<<<<<< HEAD
}

TEST(BridgeConfig, InvalidSetTypes)
=======
  EXPECT_EQ(
    "Could not parse entry: topic_name and gz_topic_name are mutually exclusive",
    g_last_log_event.message);
}

TEST_F(BridgeConfig, InvalidSetTypes)
>>>>>>> 63b651a (Garden EOL (#662))
{
  // Both ros_type_name and gz_type_name must be set
  auto yaml = R"(
- topic_name: foo
  ros_type_name: bar)";

  auto results = ros_gz_bridge::readFromYamlString(yaml);
  EXPECT_EQ(0u, results.size());
<<<<<<< HEAD
}

TEST(BridgeConfig, ParseDirection)
=======
  EXPECT_EQ(
    "Could not parse entry: both ros_type_name and gz_type_name must be set",
    g_last_log_event.message);
}

TEST_F(BridgeConfig, ParseDirection)
>>>>>>> 63b651a (Garden EOL (#662))
{
  {
    // Check that default is bidirectional
    auto yaml =
      R"(
  - topic_name: foo
    ros_type_name: std_msgs/msg/String
    gz_type_name: ignition.msgs.StringMsg)";

    auto results = ros_gz_bridge::readFromYamlString(yaml);
    EXPECT_EQ(ros_gz_bridge::BridgeDirection::BIDIRECTIONAL, results[0].direction);
  }

  {
    auto yaml =
      R"(
  - topic_name: foo
    ros_type_name: std_msgs/msg/String
    gz_type_name: ignition.msgs.StringMsg
    direction: BIDIRECTIONAL
    )";

    auto results = ros_gz_bridge::readFromYamlString(yaml);
    EXPECT_EQ(ros_gz_bridge::BridgeDirection::BIDIRECTIONAL, results[0].direction);
  }

  {
    auto yaml =
      R"(
  - topic_name: foo
    ros_type_name: std_msgs/msg/String
    gz_type_name: ignition.msgs.StringMsg
    direction: ROS_TO_GZ
    )";

    auto results = ros_gz_bridge::readFromYamlString(yaml);
    EXPECT_EQ(ros_gz_bridge::BridgeDirection::ROS_TO_GZ, results[0].direction);
  }

  {
    auto yaml =
      R"(
  - topic_name: foo
    ros_type_name: std_msgs/msg/String
    gz_type_name: ignition.msgs.StringMsg
    direction: GZ_TO_ROS
    )";

    auto results = ros_gz_bridge::readFromYamlString(yaml);
    EXPECT_EQ(ros_gz_bridge::BridgeDirection::GZ_TO_ROS, results[0].direction);
  }


  {
    auto yaml =
      R"(
  - topic_name: foo
    ros_type_name: std_msgs/msg/String
    gz_type_name: ignition.msgs.StringMsg
<<<<<<< HEAD
    direction: asdfasdfasdfasdf
=======
    direction: foobar
>>>>>>> 63b651a (Garden EOL (#662))
    )";

    auto results = ros_gz_bridge::readFromYamlString(yaml);
    EXPECT_EQ(0u, results.size());
<<<<<<< HEAD
  }
}
=======
    EXPECT_EQ("Could not parse entry: invalid direction [foobar]", g_last_log_event.message);
  }
}

TEST_F(BridgeConfig, InvalidFileDoesntExist)
{
  auto results = ros_gz_bridge::readFromYamlFile("this/should/never/be/a/file.yaml");
  EXPECT_EQ(0u, results.size());
  EXPECT_EQ(
    "Could not parse config: failed to open file [this/should/never/be/a/file.yaml]",
    g_last_log_event.message);
}

TEST_F(BridgeConfig, InvalidTopLevel)
{
  auto results = ros_gz_bridge::readFromYamlFile("test/config/invalid.yaml");
  EXPECT_EQ(0u, results.size());
  EXPECT_EQ(
    "Could not parse config: top level must be a YAML sequence",
    g_last_log_event.message);
}

TEST_F(BridgeConfig, EmptyYAML)
{
  auto results = ros_gz_bridge::readFromYamlFile("test/config/empty.yaml");
  EXPECT_EQ(0u, results.size());
  EXPECT_EQ(
    "Could not parse config: file empty [test/config/empty.yaml]",
    g_last_log_event.message);
}
>>>>>>> 63b651a (Garden EOL (#662))
