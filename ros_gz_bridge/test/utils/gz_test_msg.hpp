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

#ifndef UTILS__GZ_TEST_MSG_HPP_
#define UTILS__GZ_TEST_MSG_HPP_

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/altimeter.pb.h>
#include <gz/msgs/any.pb.h>
#include <gz/msgs/axis.pb.h>
#include <gz/msgs/battery_state.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/color.pb.h>
#include <gz/msgs/contact.pb.h>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_wrench.pb.h>
<<<<<<< HEAD
#include <gz/msgs/float.pb.h>
#include <gz/msgs/float_v.pb.h>
=======
#include <gz/msgs/dataframe.pb.h>
#include <gz/msgs/float.pb.h>
>>>>>>> 63b651a (Garden EOL (#662))
#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/gui_camera.pb.h>
#include <gz/msgs/header.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/int32.pb.h>
#include <gz/msgs/joint_trajectory.pb.h>
#include <gz/msgs/joint_wrench.pb.h>
#include <gz/msgs/joy.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/light.pb.h>
#include <gz/msgs/magnetometer.pb.h>
<<<<<<< HEAD
=======
#include <gz/msgs/material_color.pb.h>
>>>>>>> 63b651a (Garden EOL (#662))
#include <gz/msgs/model.pb.h>
#include <gz/msgs/navsat.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/odometry_with_covariance.pb.h>
#include <gz/msgs/param.pb.h>
#include <gz/msgs/param_v.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_with_covariance.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/quaternion.pb.h>
#include <gz/msgs/sensor_noise.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>
#include <gz/msgs/track_visual.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/twist_with_covariance.pb.h>
#include <gz/msgs/uint32.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/video_record.pb.h>
#include <gz/msgs/wrench.pb.h>
#include <gz/msgs/annotated_axis_aligned_2d_box_v.pb.h>
#include <gz/msgs/annotated_oriented_3d_box_v.pb.h>

#include <memory>

#include <ros_gz_bridge/ros_gz_bridge.hpp>
<<<<<<< HEAD
#if HAVE_DATAFRAME
#include <gz/msgs/dataframe.pb.h>
#endif  // HAVE_DATAFRAME

#if HAVE_MATERIALCOLOR
#include <gz/msgs/material_color.pb.h>
#endif  // HAVE_MATERIALCOLOR
=======
>>>>>>> 63b651a (Garden EOL (#662))

namespace ros_gz_bridge
{
namespace testing
{
/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Any & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Any> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Boolean & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Boolean> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Color & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Color> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Empty & _msg);

/// \brief Compare a message with the populated for testing. Noop for Empty
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Empty> &);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Float & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Float> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Float_V & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Float_V> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Double & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Double> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Int32 & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Int32> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::UInt32 & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::UInt32> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Header & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Header> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Clock & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Clock> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
<<<<<<< HEAD
void createTestMsg(gz::msgs::StringMsg & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
=======
>>>>>>> 63b651a (Garden EOL (#662))
void createTestMsg(gz::msgs::SensorNoise & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::SensorNoise> & _msg);

<<<<<<< HEAD
=======
/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::StringMsg & _msg);

>>>>>>> 63b651a (Garden EOL (#662))
/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::StringMsg> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Quaternion & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Quaternion> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Vector3d & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Vector3d> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Param_V & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Param_V> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Param & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Param> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Pose & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Pose> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::PoseWithCovariance & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::PoseWithCovariance> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Pose_V & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Pose_V> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Twist & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Twist> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::TwistWithCovariance & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::TwistWithCovariance> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Wrench & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Wrench> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::JointWrench & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::JointWrench> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Altimeter & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Altimeter> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Joy & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Joy> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Entity & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Entity> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::EntityWrench & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::EntityWrench> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Contact & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Contact> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Contacts & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Contacts> & _msg);

<<<<<<< HEAD
#if HAVE_DATAFRAME
=======
>>>>>>> 63b651a (Garden EOL (#662))
/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Dataframe & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Dataframe> & _msg);
<<<<<<< HEAD
#endif  // HAVE_DATAFRAME
=======
>>>>>>> 63b651a (Garden EOL (#662))

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Image & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Image> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::CameraInfo & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::CameraInfo> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::FluidPressure & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::FluidPressure> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::IMU & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::IMU> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Axis & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Axis> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Model & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Model> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::LaserScan & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::LaserScan> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Magnetometer & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Magnetometer> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::NavSat & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::NavSat> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Actuators & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Actuators> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Odometry & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Odometry> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::OdometryWithCovariance & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::OdometryWithCovariance> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::PointCloudPacked & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::PointCloudPacked> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::BatteryState & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::BatteryState> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::JointTrajectoryPoint & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::JointTrajectoryPoint> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::JointTrajectory & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::JointTrajectory> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Light & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Light> & _msg);

<<<<<<< HEAD
#if HAVE_MATERIALCOLOR
=======
>>>>>>> 63b651a (Garden EOL (#662))
/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::MaterialColor & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::MaterialColor> & _msg);
<<<<<<< HEAD
#endif  // HAVE_MATERIALCOLOR
=======
>>>>>>> 63b651a (Garden EOL (#662))

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::GUICamera & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::GUICamera> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::StringMsg_V & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::StringMsg_V> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::Time & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::Time> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::TrackVisual & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::TrackVisual> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::VideoRecord & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::VideoRecord> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::AnnotatedAxisAligned2DBox & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::AnnotatedAxisAligned2DBox> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::AnnotatedAxisAligned2DBox_V & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::AnnotatedAxisAligned2DBox_V> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::AnnotatedOriented3DBox & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::AnnotatedOriented3DBox> & _msg);

/// \brief Create a message used for testing.
/// \param[out] _msg The message populated.
void createTestMsg(gz::msgs::AnnotatedOriented3DBox_V & _msg);

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::AnnotatedOriented3DBox_V> & _msg);

}  // namespace testing
}  // namespace ros_gz_bridge

#endif  // UTILS__GZ_TEST_MSG_HPP_
