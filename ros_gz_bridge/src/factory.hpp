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

#ifndef FACTORY_HPP_
#define FACTORY_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <type_traits>

#include <gz/transport/Node.hh>
#include <gz/transport/SubscribeOptions.hh>

// include ROS 2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_options.hpp>

#include "factory_interface.hpp"

template<class T, class = void>
struct has_header : std::false_type
{
};

template<class T>
struct has_header<T, std::void_t<decltype(T::header)>>: std::true_type
{
};

namespace ros_gz_bridge
{

template<typename ROS_T, typename GZ_T>
class Factory : public FactoryInterface
{
public:
  Factory(
    const std::string & ros_type_name, const std::string & gz_type_name)
  : ros_type_name_(ros_type_name),
    gz_type_name_(gz_type_name)
  {}

  virtual ~Factory() {}

  rclcpp::PublisherBase::SharedPtr
  create_ros_publisher(
    rclcpp::Node::SharedPtr ros_node,
    const std::string & topic_name,
    size_t queue_size)
  {
    // Allow QoS overriding
    auto options = rclcpp::PublisherOptions();
    options.qos_overriding_options = rclcpp::QosOverridingOptions {
      {
        rclcpp::QosPolicyKind::Deadline,
        rclcpp::QosPolicyKind::Depth,
        rclcpp::QosPolicyKind::Durability,
        rclcpp::QosPolicyKind::History,
        rclcpp::QosPolicyKind::Liveliness,
        rclcpp::QosPolicyKind::Reliability
      },
    };

    std::shared_ptr<rclcpp::Publisher<ROS_T>> publisher =
      ros_node->create_publisher<ROS_T>(
      topic_name, rclcpp::QoS(rclcpp::KeepLast(queue_size)), options);
    return publisher;
  }

  gz::transport::Node::Publisher
  create_gz_publisher(
    std::shared_ptr<gz::transport::Node> gz_node,
    const std::string & topic_name,
    size_t /*queue_size*/)
  {
    return gz_node->Advertise<GZ_T>(topic_name);
  }

  rclcpp::SubscriptionBase::SharedPtr
  create_ros_subscriber(
    rclcpp::Node::SharedPtr ros_node,
    const std::string & topic_name,
    size_t queue_size,
    gz::transport::Node::Publisher & gz_pub)
  {
    std::function<void(std::shared_ptr<const ROS_T>)> fn = std::bind(
      &Factory<ROS_T, GZ_T>::ros_callback,
      std::placeholders::_1, gz_pub,
      ros_type_name_, gz_type_name_,
      ros_node);
    auto options = rclcpp::SubscriptionOptions();
    // Ignore messages that are published from this bridge.
    options.ignore_local_publications = true;
    // Allow QoS overriding
    options.qos_overriding_options =
      rclcpp::QosOverridingOptions::with_default_policies();
    std::shared_ptr<rclcpp::Subscription<ROS_T>> subscription =
      ros_node->create_subscription<ROS_T>(
      topic_name, rclcpp::QoS(rclcpp::KeepLast(queue_size)), fn, options);
    return subscription;
  }

  void
  create_gz_subscriber(
    std::shared_ptr<gz::transport::Node> node,
    const std::string & topic_name,
    size_t /*queue_size*/,
    rclcpp::PublisherBase::SharedPtr ros_pub,
    bool override_timestamps_with_wall_time)
  {
    auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<ROS_T>>(ros_pub);
    if (pub == nullptr) {
      return;
    }
    std::function<void(const GZ_T &)> subCb =
      [this, pub, override_timestamps_with_wall_time](const GZ_T & _msg)
      {
        this->gz_callback(_msg, pub, override_timestamps_with_wall_time);
      };

    // Ignore messages that are published from this bridge.
    gz::transport::SubscribeOptions opts;
    opts.SetIgnoreLocalMessages(true);
    node->Subscribe(topic_name, subCb, opts);
  }

protected:
  static
  void ros_callback(
    std::shared_ptr<const ROS_T> ros_msg,
    gz::transport::Node::Publisher & gz_pub,
    const std::string & ros_type_name,
    const std::string & gz_type_name,
    rclcpp::Node::SharedPtr ros_node)
  {
    GZ_T gz_msg;
    convert_ros_to_gz(*ros_msg, gz_msg);
    gz_pub.Publish(gz_msg);
    RCLCPP_INFO_ONCE(
      ros_node->get_logger(),
      "Passing message from ROS %s to Gazebo %s (showing msg only once per type)",
      ros_type_name.c_str(), gz_type_name.c_str());
  }

  static
  void gz_callback(
    const GZ_T & gz_msg,
    std::shared_ptr<rclcpp::Publisher<ROS_T>> ros_pub,
    bool override_timestamps_with_wall_time)
  {
    ROS_T ros_msg;
    convert_gz_to_ros(gz_msg, ros_msg);
    if constexpr (has_header<ROS_T>::value) {
      if (override_timestamps_with_wall_time) {
        auto now = std::chrono::system_clock::now().time_since_epoch();
        auto ns =
          std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
        ros_msg.header.stamp.sec = ns / 1e9;
        ros_msg.header.stamp.nanosec = ns - ros_msg.header.stamp.sec * 1e9;
      }
    }
    ros_pub->publish(ros_msg);
  }

public:
  // since convert functions call each other for sub messages they must be
  // public defined outside of the class
  static
  void
  convert_ros_to_gz(
    const ROS_T & ros_msg,
    GZ_T & gz_msg);
  static
  void
  convert_gz_to_ros(
    const GZ_T & gz_msg,
    ROS_T & ros_msg);

  std::string ros_type_name_;
  std::string gz_type_name_;
};

}  // namespace ros_gz_bridge

#endif  // FACTORY_HPP_
