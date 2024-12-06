// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef UTILS__ROS_TEST_MSG_HPP_
#define UTILS__ROS_TEST_MSG_HPP_

#include <memory>
#include <ros_gz_bridge/ros_gz_bridge.hpp>

#include <builtin_interfaces/msg/time.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <actuator_msgs/msg/actuators.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
<<<<<<< HEAD
=======
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
>>>>>>> 63b651a (Garden EOL (#662))
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <gps_msgs/msg/gps_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ros_gz_interfaces/msg/altimeter.hpp>
#include <ros_gz_interfaces/msg/entity.hpp>
#include <ros_gz_interfaces/msg/entity_wrench.hpp>
#include <ros_gz_interfaces/msg/gui_camera.hpp>
#include <ros_gz_interfaces/msg/joint_wrench.hpp>
#include <ros_gz_interfaces/msg/contact.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <ros_gz_interfaces/msg/float32_array.hpp>
<<<<<<< HEAD
#if HAVE_DATAFRAME
#include <ros_gz_interfaces/msg/dataframe.hpp>
#endif  // HAVE_DATAFRAME
#include <ros_gz_interfaces/msg/light.hpp>
#if HAVE_MATERIALCOLOR
#include <ros_gz_interfaces/msg/material_color.hpp>
#endif  // HAVE_MATERIALCOLOR
=======
#include <ros_gz_interfaces/msg/dataframe.hpp>
#include <ros_gz_interfaces/msg/light.hpp>
#include <ros_gz_interfaces/msg/material_color.hpp>
>>>>>>> 63b651a (Garden EOL (#662))
#include <ros_gz_interfaces/msg/param_vec.hpp>
#include <ros_gz_interfaces/msg/sensor_noise.hpp>
#include <ros_gz_interfaces/msg/string_vec.hpp>
#include <ros_gz_interfaces/msg/track_visual.hpp>
#include <ros_gz_interfaces/msg/video_record.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace ros_gz_bridge
{
namespace testing
{

//////////////////////////////////////////////////
/// ROS test utils
//////////////////////////////////////////////////

/// builtin_interfaces

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(builtin_interfaces::msg::Time & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<builtin_interfaces::msg::Time> & _msg);

/// std_msgs

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(std_msgs::msg::Bool & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<std_msgs::msg::Bool> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(std_msgs::msg::ColorRGBA & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<std_msgs::msg::ColorRGBA> & _msg);

/// \brief Create a message used for testing. Noop for Empty
/// \param[out] _msg The message populated.
void createTestMsg(std_msgs::msg::Empty & _msg);

/// \brief Compare a message with the populated for testing. Noop for Empty
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<std_msgs::msg::Empty> &);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(std_msgs::msg::Float32 & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<std_msgs::msg::Float32> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(std_msgs::msg::Float64 & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<std_msgs::msg::Float64> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(std_msgs::msg::Int32 & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<std_msgs::msg::Int32> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(std_msgs::msg::UInt32 & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<std_msgs::msg::UInt32> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(std_msgs::msg::Header & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<std_msgs::msg::Header> & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std_msgs::msg::Header & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(std_msgs::msg::String & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<std_msgs::msg::String> & _msg);

/// rosgraph_msgs

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(rosgraph_msgs::msg::Clock & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<rosgraph_msgs::msg::Clock> & _msg);

/// actuator_msgs

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(actuator_msgs::msg::Actuators & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<actuator_msgs::msg::Actuators> & _msg);

/// geometry_msgs

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::Quaternion & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const geometry_msgs::msg::Quaternion & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Quaternion> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::Vector3 & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Vector3> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::Point & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Point> & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const geometry_msgs::msg::Point & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const geometry_msgs::msg::Pose & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::Pose & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Pose> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::PoseArray & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::PoseArray> & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const geometry_msgs::msg::PoseWithCovariance & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::PoseWithCovariance & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::PoseWithCovariance> & _msg);

<<<<<<< HEAD
=======
/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const geometry_msgs::msg::PoseWithCovarianceStamped & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::PoseWithCovarianceStamped & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> & _msg);

>>>>>>> 63b651a (Garden EOL (#662))
/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::PoseStamped & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::PoseStamped> & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const geometry_msgs::msg::PoseStamped & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::Transform & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Transform> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::TransformStamped & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::TransformStamped> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::Twist & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Twist> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::TwistStamped & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::TwistStamped> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::TwistWithCovariance & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::TwistWithCovariance> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::TwistWithCovarianceStamped & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::Wrench & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Wrench> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(geometry_msgs::msg::WrenchStamped & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> & _msg);

/// gps_msgs

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gps_msgs::msg::GPSFix & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gps_msgs::msg::GPSFix> & _msg);

/// tf2_msgs

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(tf2_msgs::msg::TFMessage & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<tf2_msgs::msg::TFMessage> & _msg);

/// nav_msgs

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(nav_msgs::msg::Odometry & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<nav_msgs::msg::Odometry> & _msg);

/// ros_gz_interfaces

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ros_gz_interfaces::msg::JointWrench & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::JointWrench> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ros_gz_interfaces::msg::Altimeter & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::Altimeter> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ros_gz_interfaces::msg::Light & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::Light> & _msg);

<<<<<<< HEAD
#if HAVE_MATERIALCOLOR
=======
>>>>>>> 63b651a (Garden EOL (#662))
/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ros_gz_interfaces::msg::MaterialColor & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::MaterialColor> & _msg);
<<<<<<< HEAD
#endif  // HAVE_MATERIALCOLOR
=======
>>>>>>> 63b651a (Garden EOL (#662))

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ros_gz_interfaces::msg::Entity & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::Entity> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ros_gz_interfaces::msg::EntityWrench & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::EntityWrench> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ros_gz_interfaces::msg::Contact & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::Contact> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ros_gz_interfaces::msg::Contacts & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::Contacts> & _msg);

<<<<<<< HEAD
#if HAVE_DATAFRAME
=======
>>>>>>> 63b651a (Garden EOL (#662))
/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ros_gz_interfaces::msg::Dataframe & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::Dataframe> & _msg);
<<<<<<< HEAD
#endif  // HAVE_DATAFRAME
=======
>>>>>>> 63b651a (Garden EOL (#662))

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ros_gz_interfaces::msg::GuiCamera & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::GuiCamera> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ros_gz_interfaces::msg::ParamVec & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::ParamVec> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ros_gz_interfaces::msg::SensorNoise & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::SensorNoise> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ros_gz_interfaces::msg::StringVec & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::StringVec> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ros_gz_interfaces::msg::TrackVisual & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::TrackVisual> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ros_gz_interfaces::msg::VideoRecord & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::VideoRecord> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(ros_gz_interfaces::msg::Float32Array & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::Float32Array> & _msg);

/// sensor_msgs

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(sensor_msgs::msg::Image & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::Image> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(sensor_msgs::msg::CameraInfo & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::CameraInfo> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(sensor_msgs::msg::FluidPressure & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::FluidPressure> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(sensor_msgs::msg::Imu & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::Imu> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(sensor_msgs::msg::JointState & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::JointState> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(sensor_msgs::msg::Joy & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::Joy> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(sensor_msgs::msg::LaserScan & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::LaserScan> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(sensor_msgs::msg::MagneticField & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::MagneticField> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(sensor_msgs::msg::NavSatFix & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::NavSatFix> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(sensor_msgs::msg::PointCloud2 & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::PointCloud2> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(sensor_msgs::msg::BatteryState & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::BatteryState> & _msg);

/// trajectory_msgs

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(trajectory_msgs::msg::JointTrajectoryPoint & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(trajectory_msgs::msg::JointTrajectory & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(rcl_interfaces::msg::ParameterValue & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<rcl_interfaces::msg::ParameterValue> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(vision_msgs::msg::Detection2DArray & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<vision_msgs::msg::Detection2DArray> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(vision_msgs::msg::Detection2D & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<vision_msgs::msg::Detection2D> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(vision_msgs::msg::Detection3DArray & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<vision_msgs::msg::Detection3DArray> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(vision_msgs::msg::Detection3D & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<vision_msgs::msg::Detection3D> & _msg);

}  // namespace testing
}  // namespace ros_gz_bridge

#endif  // UTILS__ROS_TEST_MSG_HPP_
