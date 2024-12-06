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

#ifndef BRIDGE_HANDLE_HPP_
#define BRIDGE_HANDLE_HPP_

#include <memory>
#include <string>

#include <gz/transport/Node.hh>
#include <rclcpp/node.hpp>

#include "get_factory.hpp"
#include <ros_gz_bridge/bridge_config.hpp>


namespace ros_gz_bridge
{

/// \brief Core functionality and data for both bridge directions
class BridgeHandle
{
public:
  /// \brief Constructor
  /// Note, does not actually start the bridge, must be done with Start()
  ///
  /// \param[in] ros_node ROS node to create publishers/subscribers on
  /// \param[in] gz_node GZ node to create publishers/subscribers on
  /// \param[in] config Configuration parameters for this handle
  BridgeHandle(
    rclcpp::Node::SharedPtr ros_node,
    std::shared_ptr<gz::transport::Node> gz_node,
    const BridgeConfig & config);

  /// \brief Destructor
  virtual ~BridgeHandle() = 0;

  /// \brief Initiate the bridge
  ///
  /// If the bridge is "lazy", then only the publisher (output) side is created.
  /// The not, both the publisher (output) and subscriber (input) side are
  /// created.
  void Start();

  /// \brief Spin the bridge, checking for new subscriptions on the output.
  ///
  /// Not necessary if the Bridge isn't lazy
  void Spin();

  /// \brief Inidicate if this is a "lazy" bridge
  ///
  /// A lazy bridge will always create the output side of the bridge, but
  /// will only create the input side of the bridge when downstream consumers
  /// are detected on the output side.
  /// This reduces the amount of overhead in the bridge processing messages
  /// for non-existent consumers.
  /// It will also allow for lazy publishers to work on the input side of the
  /// bridge.
  /// \return true if lazy
  bool IsLazy() const;

protected:
  /// \brief Get the number of subscriptions on the output side of the bridge
  virtual size_t NumSubscriptions() const = 0;

  /// \brief Indicate if the publisher (output) side has been created
  virtual bool HasPublisher() const = 0;

  /// \brief Start the publisher (output) side of the bridge
  virtual void StartPublisher() = 0;

  /// \brief Indicate if the subscriber (input) side has been created
  virtual bool HasSubscriber() const = 0;

  /// \brief Start the subscriber (input) side of the bridge
  virtual void StartSubscriber() = 0;

  /// \brief Stop the subscriber (input) side of the bridge
  virtual void StopSubscriber() = 0;

protected:
  /// \brief The ROS node used to create publishers/subscriptions
  rclcpp::Node::SharedPtr ros_node_;

  /// \brief The Gazebo node used to create publishers/subscriptions
  std::shared_ptr<gz::transport::Node> gz_node_;

  /// \brief The configuration parameters of this bridge
  BridgeConfig config_;

  /// \brief Typed factory used to create publishers/subscribers
  std::shared_ptr<FactoryInterface> factory_;
<<<<<<< HEAD
=======

  /// \brief Override the header.stamp field of the outgoing messages with the wall time
  bool override_timestamps_with_wall_time_ = false;
>>>>>>> 63b651a (Garden EOL (#662))
};

}  // namespace ros_gz_bridge
#endif  // BRIDGE_HANDLE_HPP_
