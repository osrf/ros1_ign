// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include <functional>
#include <thread>
#include <gz/common/Console.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/SystemLoader.hh>
#include <gz/sim/ServerConfig.hh>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace ros_gz_sim
{
class GzServer : public rclcpp::Node
{
public:
  // Class constructor.
  explicit GzServer(const rclcpp::NodeOptions & options)
  : Node("gzserver", options)
  {
    thread_ = std::thread(std::bind(&GzServer::OnStart, this));
  }

public:
  // Class destructor.
  ~GzServer()
  {
    // Make sure to join the thread on shutdown.
    if (thread_.joinable()) {
      thread_.join();
    }
  }

public:
  /// \brief Run the gz sim server.
  void OnStart()
  {
    auto world_sdf_file = this->declare_parameter("world_sdf_file", "");
    auto world_sdf_string = this->declare_parameter("world_sdf_string", "");

    gz::common::Console::SetVerbosity(4);
    gz::sim::ServerConfig server_config;

    if (!world_sdf_file.empty()) {
      server_config.SetSdfFile(world_sdf_file);
    } else if (!world_sdf_string.empty()) {
      server_config.SetSdfString(world_sdf_string);
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Must specify either 'world_sdf_file' or 'world_sdf_string'");
      return;
    }

    // (azeey) Testing if a plugin can be loaded along side the defaults. Not working yet.

    // <plugin
    //   filename="gz-sim-imu-system"
    //   name="gz::sim::systems::Imu">
    // </plugin>
    sdf::Plugin imu_sdf_plugin;
    imu_sdf_plugin.SetName("gz::sim::systems::Imu");
    imu_sdf_plugin.SetFilename("gz-sim-imu-system");
    gz::sim::SystemLoader loader;
    auto imu_plugin = loader.LoadPlugin(imu_sdf_plugin);
    std::cout << imu_plugin.value() << std::endl;

    gz::sim::Server server(server_config);
    server.RunOnce();
    if (!server.AddSystem(*imu_plugin)) {
      RCLCPP_ERROR(this->get_logger(), "IMU system not added");
    }
    server.Run(true /*blocking*/, 0, false /*paused*/);
  }

private:
  /// \brief We don't want to block the ROS thread.
  std::thread thread_;
};
}  // namespace ros_gz_sim

RCLCPP_COMPONENTS_REGISTER_NODE(ros_gz_sim::GzServer)