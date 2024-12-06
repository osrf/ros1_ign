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

#include "ros_test_msg.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <cstddef>

#include "gz/msgs/config.hh"

#if GZ_MSGS_MAJOR_VERSION >= 10
#define GZ_MSGS_IMU_HAS_COVARIANCE
#endif

namespace ros_gz_bridge
{
namespace testing
{
void createTestMsg(builtin_interfaces::msg::Time & _msg)
{
  _msg.sec = 12;
  _msg.nanosec = 150;
}

void compareTestMsg(const std::shared_ptr<builtin_interfaces::msg::Time> & _msg)
{
  builtin_interfaces::msg::Time expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.sec, _msg->sec);
  EXPECT_EQ(expected_msg.nanosec, _msg->nanosec);
}

void createTestMsg(std_msgs::msg::Bool & _msg)
{
  _msg.data = true;
}

void compareTestMsg(const std::shared_ptr<std_msgs::msg::Bool> & _msg)
{
  std_msgs::msg::Bool expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.data, _msg->data);
}

<<<<<<< HEAD
=======
void createTestMsg(std_msgs::msg::ColorRGBA & _msg)
{
  _msg.r = 0.2;
  _msg.g = 0.4;
  _msg.b = 0.6;
  _msg.a = 0.8;
}

>>>>>>> 63b651a (Garden EOL (#662))
void createTestMsg(actuator_msgs::msg::Actuators & _msg)
{
  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);

  _msg.header = header_msg;
  _msg.position = {0.5, 0.5, 0.5};
  _msg.velocity = {1.0, 1.0, 1.0};
  _msg.normalized = {0.2, 0.2, 0.2};
}

void compareTestMsg(const std::shared_ptr<actuator_msgs::msg::Actuators> & _msg)
{
  actuator_msgs::msg::Actuators expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);

  for (auto i = 0u; i < _msg->position.size(); ++i) {
    EXPECT_FLOAT_EQ(expected_msg.position[i], _msg->position[i]);
  }

  for (auto i = 0u; i < _msg->velocity.size(); ++i) {
    EXPECT_FLOAT_EQ(expected_msg.velocity[i], _msg->velocity[i]);
  }

  for (auto i = 0u; i < _msg->normalized.size(); ++i) {
    EXPECT_FLOAT_EQ(expected_msg.normalized[i], _msg->normalized[i]);
  }
}

<<<<<<< HEAD
void createTestMsg(std_msgs::msg::ColorRGBA & _msg)
{
  _msg.r = 0.2;
  _msg.g = 0.4;
  _msg.b = 0.6;
  _msg.a = 0.8;
}

=======
>>>>>>> 63b651a (Garden EOL (#662))
void compareTestMsg(const std::shared_ptr<std_msgs::msg::ColorRGBA> & _msg)
{
  std_msgs::msg::ColorRGBA expected_msg;
  createTestMsg(expected_msg);

  EXPECT_FLOAT_EQ(expected_msg.r, _msg->r);
  EXPECT_FLOAT_EQ(expected_msg.g, _msg->g);
  EXPECT_FLOAT_EQ(expected_msg.b, _msg->b);
  EXPECT_FLOAT_EQ(expected_msg.a, _msg->a);
}

void createTestMsg(std_msgs::msg::Empty &)
{
}

void compareTestMsg(const std::shared_ptr<std_msgs::msg::Empty> &)
{
}

void createTestMsg(std_msgs::msg::Float32 & _msg)
{
  _msg.data = 1.5;
}

void compareTestMsg(const std::shared_ptr<std_msgs::msg::Float32> & _msg)
{
  std_msgs::msg::Float32 expected_msg;
  createTestMsg(expected_msg);

  EXPECT_FLOAT_EQ(expected_msg.data, _msg->data);
}

void createTestMsg(ros_gz_interfaces::msg::Float32Array & _msg)
{
  std_msgs::msg::Float32 msg;
  createTestMsg(msg);
  _msg.data.push_back(msg.data);
}

void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::Float32Array> & _msg)
{
  ros_gz_interfaces::msg::Float32Array expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.data.size(), _msg->data.size());
  EXPECT_EQ(1u, _msg->data.size());
  EXPECT_FLOAT_EQ(expected_msg.data[0], _msg->data[0]);
}

void createTestMsg(std_msgs::msg::Float64 & _msg)
{
  _msg.data = 1.5;
}

void compareTestMsg(const std::shared_ptr<std_msgs::msg::Float64> & _msg)
{
  std_msgs::msg::Float64 expected_msg;
  createTestMsg(expected_msg);

  EXPECT_FLOAT_EQ(expected_msg.data, _msg->data);
}

void createTestMsg(std_msgs::msg::Int32 & _msg)
{
  _msg.data = -10;
}

void compareTestMsg(const std::shared_ptr<std_msgs::msg::Int32> & _msg)
{
  std_msgs::msg::Int32 expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.data, _msg->data);
}

void createTestMsg(std_msgs::msg::UInt32 & _msg)
{
  _msg.data = 1;
}

void compareTestMsg(const std::shared_ptr<std_msgs::msg::UInt32> & _msg)
{
  std_msgs::msg::UInt32 expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.data, _msg->data);
}

void createTestMsg(std_msgs::msg::Header & _msg)
{
  // _msg.seq        = 1;
  _msg.stamp.sec = 2;
  _msg.stamp.nanosec = 3;
  _msg.frame_id = "frame_id_value";
}

void compareTestMsg(const std::shared_ptr<std_msgs::msg::Header> & _msg)
{
  std_msgs::msg::Header expected_msg;
  createTestMsg(expected_msg);

  // EXPECT_GE(expected_msg.seq,        0u);
  EXPECT_EQ(expected_msg.stamp.sec, _msg->stamp.sec);
  EXPECT_EQ(expected_msg.stamp.nanosec, _msg->stamp.nanosec);
  EXPECT_EQ(expected_msg.frame_id, _msg->frame_id);
}

void compareTestMsg(const std_msgs::msg::Header & _msg)
{
  std_msgs::msg::Header expected_msg;
  createTestMsg(expected_msg);

  // EXPECT_GE(expected_msg.seq,        0u);
  EXPECT_EQ(expected_msg.stamp.sec, _msg.stamp.sec);
  EXPECT_EQ(expected_msg.stamp.nanosec, _msg.stamp.nanosec);
  EXPECT_EQ(expected_msg.frame_id, _msg.frame_id);
}

void createTestMsg(std_msgs::msg::String & _msg)
{
  _msg.data = "string";
}

void compareTestMsg(const std::shared_ptr<std_msgs::msg::String> & _msg)
{
  std_msgs::msg::String expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.data, _msg->data);
}

void createTestMsg(geometry_msgs::msg::Quaternion & _msg)
{
  _msg.x = 1;
  _msg.y = 2;
  _msg.z = 3;
  _msg.w = 4;
}

void compareTestMsg(const geometry_msgs::msg::Quaternion & _msg)
{
  geometry_msgs::msg::Quaternion expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.x, _msg.x);
  EXPECT_EQ(expected_msg.y, _msg.y);
  EXPECT_EQ(expected_msg.z, _msg.z);
  EXPECT_EQ(expected_msg.w, _msg.w);
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Quaternion> & _msg)
{
  geometry_msgs::msg::Quaternion expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.x, _msg->x);
  EXPECT_EQ(expected_msg.y, _msg->y);
  EXPECT_EQ(expected_msg.z, _msg->z);
  EXPECT_EQ(expected_msg.w, _msg->w);
}

void createTestMsg(geometry_msgs::msg::Vector3 & _msg)
{
  _msg.x = 1;
  _msg.y = 2;
  _msg.z = 3;
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Vector3> & _msg)
{
  geometry_msgs::msg::Vector3 expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.x, _msg->x);
  EXPECT_EQ(expected_msg.y, _msg->y);
  EXPECT_EQ(expected_msg.z, _msg->z);
}

void createTestMsg(rosgraph_msgs::msg::Clock & _msg)
{
  _msg.clock.sec = 1;
  _msg.clock.nanosec = 2;
}

void createTestMsg(geometry_msgs::msg::Point & _msg)
{
  _msg.x = 1;
  _msg.y = 2;
  _msg.z = 3;
}

void compareTestMsg(const std::shared_ptr<rosgraph_msgs::msg::Clock> & _msg)
{
  rosgraph_msgs::msg::Clock expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.clock.sec, _msg->clock.sec);
  EXPECT_EQ(expected_msg.clock.nanosec, _msg->clock.nanosec);
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Point> & _msg)
{
  geometry_msgs::msg::Point expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.x, _msg->x);
  EXPECT_EQ(expected_msg.y, _msg->y);
  EXPECT_EQ(expected_msg.z, _msg->z);
}

void compareTestMsg(const geometry_msgs::msg::Point & _msg)
{
  geometry_msgs::msg::Point expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.x, _msg.x);
  EXPECT_EQ(expected_msg.y, _msg.y);
  EXPECT_EQ(expected_msg.z, _msg.z);
}

void compareTestMsg(const geometry_msgs::msg::Pose & _msg)
{
  compareTestMsg(_msg.position);
  compareTestMsg(_msg.orientation);
}

void createTestMsg(geometry_msgs::msg::Pose & _msg)
{
  createTestMsg(_msg.position);
  createTestMsg(_msg.orientation);
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Pose> & _msg)
{
  compareTestMsg(_msg->position);
  compareTestMsg(_msg->orientation);
}

void createTestMsg(geometry_msgs::msg::PoseArray & _msg)
{
  createTestMsg(_msg.header);

  geometry_msgs::msg::Pose pose;
  createTestMsg(pose);
  _msg.poses.push_back(pose);
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::PoseArray> & _msg)
{
  compareTestMsg(_msg->header);

  geometry_msgs::msg::PoseArray expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<geometry_msgs::msg::Pose>(_msg->poses[0]));
}

void compareTestMsg(const geometry_msgs::msg::PoseWithCovariance & _msg)
{
  compareTestMsg(_msg.pose.position);
  compareTestMsg(_msg.pose.orientation);
  for (int i = 0; i < 36; i++) {
    EXPECT_EQ(_msg.covariance[i], i);
  }
}

void createTestMsg(geometry_msgs::msg::PoseWithCovariance & _msg)
{
  createTestMsg(_msg.pose.position);
  createTestMsg(_msg.pose.orientation);
  for (int i = 0; i < 36; i++) {
    _msg.covariance[i] = i;
  }
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::PoseWithCovariance> & _msg)
{
  compareTestMsg(_msg->pose.position);
  compareTestMsg(_msg->pose.orientation);
  for (int i = 0; i < 36; i++) {
    EXPECT_EQ(_msg->covariance[i], i);
  }
}

<<<<<<< HEAD
=======
void createTestMsg(geometry_msgs::msg::PoseWithCovarianceStamped & _msg)
{
  createTestMsg(_msg.header);
  createTestMsg(_msg.pose);
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> & _msg)
{
  compareTestMsg(std::make_shared<geometry_msgs::msg::PoseWithCovariance>(_msg->pose));
  compareTestMsg(std::make_shared<std_msgs::msg::Header>(_msg->header));
}

>>>>>>> 63b651a (Garden EOL (#662))
void createTestMsg(geometry_msgs::msg::PoseStamped & _msg)
{
  createTestMsg(_msg.header);
  createTestMsg(_msg.pose);
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::PoseStamped> & _msg)
{
  compareTestMsg(_msg->header);
  compareTestMsg(_msg->pose);
}

void compareTestMsg(const geometry_msgs::msg::PoseStamped & _msg)
{
  compareTestMsg(_msg.header);
  compareTestMsg(_msg.pose);
}

void createTestMsg(geometry_msgs::msg::Transform & _msg)
{
  createTestMsg(_msg.translation);
  createTestMsg(_msg.rotation);
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Transform> & _msg)
{
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->translation));
  compareTestMsg(std::make_shared<geometry_msgs::msg::Quaternion>(_msg->rotation));
}
void createTestMsg(geometry_msgs::msg::TransformStamped & _msg)
{
  createTestMsg(_msg.header);
  createTestMsg(_msg.transform);
  _msg.child_frame_id = "child_frame_id_value";
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::TransformStamped> & _msg)
{
  geometry_msgs::msg::TransformStamped expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  compareTestMsg(std::make_shared<geometry_msgs::msg::Transform>(_msg->transform));
  EXPECT_EQ(expected_msg.child_frame_id, _msg->child_frame_id);
}

void createTestMsg(tf2_msgs::msg::TFMessage & _msg)
{
  geometry_msgs::msg::TransformStamped tf;
  createTestMsg(tf);
  _msg.transforms.push_back(tf);
}

void compareTestMsg(const std::shared_ptr<tf2_msgs::msg::TFMessage> & _msg)
{
  tf2_msgs::msg::TFMessage expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<geometry_msgs::msg::TransformStamped>(_msg->transforms[0]));
}

void createTestMsg(geometry_msgs::msg::Twist & _msg)
{
  createTestMsg(_msg.linear);
  createTestMsg(_msg.angular);
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Twist> & _msg)
{
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->linear));
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->angular));
}

void createTestMsg(geometry_msgs::msg::TwistStamped & _msg)
{
  createTestMsg(_msg.header);
  createTestMsg(_msg.twist);
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::TwistStamped> & _msg)
{
  compareTestMsg(std::make_shared<geometry_msgs::msg::Twist>(_msg->twist));
  compareTestMsg(std::make_shared<std_msgs::msg::Header>(_msg->header));
}

void createTestMsg(geometry_msgs::msg::TwistWithCovariance & _msg)
{
  createTestMsg(_msg.twist.linear);
  createTestMsg(_msg.twist.angular);
  for (int i = 0; i < 36; i++) {
    _msg.covariance[i] = i;
  }
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::TwistWithCovariance> & _msg)
{
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->twist.linear));
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->twist.angular));
  for (int i = 0; i < 36; i++) {
    EXPECT_EQ(_msg->covariance[i], i);
  }
}

void createTestMsg(geometry_msgs::msg::TwistWithCovarianceStamped & _msg)
{
  createTestMsg(_msg.header);
  createTestMsg(_msg.twist);
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped> & _msg)
{
  compareTestMsg(std::make_shared<geometry_msgs::msg::TwistWithCovariance>(_msg->twist));
  compareTestMsg(std::make_shared<std_msgs::msg::Header>(_msg->header));
}

void createTestMsg(geometry_msgs::msg::Wrench & _msg)
{
  createTestMsg(_msg.force);
  createTestMsg(_msg.torque);
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::Wrench> & _msg)
{
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->force));
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->torque));
}

void createTestMsg(geometry_msgs::msg::WrenchStamped & _msg)
{
  createTestMsg(_msg.header);
  createTestMsg(_msg.wrench.force);
  createTestMsg(_msg.wrench.torque);
}

void compareTestMsg(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> & _msg)
{
  compareTestMsg(std::make_shared<std_msgs::msg::Header>(_msg->header));
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->wrench.force));
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->wrench.torque));
}

void createTestMsg(gps_msgs::msg::GPSFix & _msg)
{
  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);

  _msg.header = header_msg;
  _msg.status.status = gps_msgs::msg::GPSStatus::STATUS_GBAS_FIX;
  _msg.latitude = 0.00;
  _msg.longitude = 0.00;
  _msg.altitude = 0.00;
  _msg.position_covariance = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  _msg.position_covariance_type = gps_msgs::msg::GPSFix::COVARIANCE_TYPE_UNKNOWN;
}

void compareTestMsg(const std::shared_ptr<gps_msgs::msg::GPSFix> & _msg)
{
  gps_msgs::msg::GPSFix expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_EQ(expected_msg.status, _msg->status);
  EXPECT_FLOAT_EQ(expected_msg.latitude, _msg->latitude);
  EXPECT_FLOAT_EQ(expected_msg.longitude, _msg->longitude);
  EXPECT_FLOAT_EQ(expected_msg.altitude, _msg->altitude);
  EXPECT_EQ(expected_msg.position_covariance_type, _msg->position_covariance_type);

  for (auto i = 0u; i < 9; ++i) {
    EXPECT_FLOAT_EQ(0, _msg->position_covariance[i]);
  }
}

void createTestMsg(ros_gz_interfaces::msg::Light & _msg)
{
  createTestMsg(_msg.header);

  _msg.name = "test_light";
  _msg.type = 1;

  createTestMsg(_msg.pose);
  createTestMsg(_msg.diffuse);
  createTestMsg(_msg.specular);
  _msg.attenuation_constant = 0.2;
  _msg.attenuation_linear = 0.4;
  _msg.attenuation_quadratic = 0.6;
  createTestMsg(_msg.direction);
  _msg.range = 25.0;
  _msg.cast_shadows = true;
  _msg.spot_inner_angle = 0.3;
  _msg.spot_outer_angle = 0.6;
  _msg.spot_falloff = 10.0;

  _msg.id = 24;

  _msg.parent_id = 6;

  _msg.intensity = 125.0;
}

void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::Light> & _msg)
{
  ros_gz_interfaces::msg::Light expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);

  EXPECT_EQ(expected_msg.name, _msg->name);
  EXPECT_EQ(expected_msg.type, _msg->type);

  compareTestMsg(std::make_shared<geometry_msgs::msg::Pose>(_msg->pose));
  compareTestMsg(std::make_shared<std_msgs::msg::ColorRGBA>(_msg->diffuse));
  compareTestMsg(std::make_shared<std_msgs::msg::ColorRGBA>(_msg->specular));
  EXPECT_FLOAT_EQ(expected_msg.attenuation_constant, _msg->attenuation_constant);
  EXPECT_FLOAT_EQ(expected_msg.attenuation_linear, _msg->attenuation_linear);
  EXPECT_FLOAT_EQ(expected_msg.attenuation_quadratic, _msg->attenuation_quadratic);
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->direction));
  EXPECT_FLOAT_EQ(expected_msg.range, _msg->range);
  EXPECT_EQ(expected_msg.cast_shadows, _msg->cast_shadows);
  EXPECT_FLOAT_EQ(expected_msg.spot_inner_angle, _msg->spot_inner_angle);
  EXPECT_FLOAT_EQ(expected_msg.spot_outer_angle, _msg->spot_outer_angle);
  EXPECT_FLOAT_EQ(expected_msg.spot_falloff, _msg->spot_falloff);

  EXPECT_EQ(expected_msg.id, _msg->id);

  EXPECT_EQ(expected_msg.parent_id, _msg->parent_id);

  EXPECT_FLOAT_EQ(expected_msg.intensity, _msg->intensity);
}

<<<<<<< HEAD
#if HAVE_MATERIALCOLOR
=======
>>>>>>> 63b651a (Garden EOL (#662))
void createTestMsg(ros_gz_interfaces::msg::MaterialColor & _msg)
{
  createTestMsg(_msg.header);
  createTestMsg(_msg.entity);
  createTestMsg(_msg.ambient);
  createTestMsg(_msg.diffuse);
  createTestMsg(_msg.specular);
  createTestMsg(_msg.emissive);
  _msg.shininess = 1.0;
  _msg.entity_match = ros_gz_interfaces::msg::MaterialColor::ALL;
}

void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::MaterialColor> & _msg)
{
  ros_gz_interfaces::msg::MaterialColor expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  compareTestMsg(std::make_shared<ros_gz_interfaces::msg::Entity>(_msg->entity));
  compareTestMsg(std::make_shared<std_msgs::msg::ColorRGBA>(_msg->ambient));
  compareTestMsg(std::make_shared<std_msgs::msg::ColorRGBA>(_msg->diffuse));
  compareTestMsg(std::make_shared<std_msgs::msg::ColorRGBA>(_msg->specular));
  compareTestMsg(std::make_shared<std_msgs::msg::ColorRGBA>(_msg->emissive));

  EXPECT_EQ(expected_msg.shininess, _msg->shininess);
  EXPECT_EQ(expected_msg.entity_match, _msg->entity_match);
}
<<<<<<< HEAD
#endif  // HAVE_MATERIALCOLOR
=======
>>>>>>> 63b651a (Garden EOL (#662))

void createTestMsg(ros_gz_interfaces::msg::GuiCamera & _msg)
{
  createTestMsg(_msg.header);
  createTestMsg(_msg.track);
  createTestMsg(_msg.pose);

  _msg.name = "test_gui_camera";
  _msg.view_controller = "test_gui_camera_view_controller";
  _msg.projection_type = "test_gui_camera_projection_type";
}

void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::GuiCamera> & _msg)
{
  ros_gz_interfaces::msg::GuiCamera expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  compareTestMsg(std::make_shared<ros_gz_interfaces::msg::TrackVisual>(_msg->track));
  compareTestMsg(std::make_shared<geometry_msgs::msg::Pose>(_msg->pose));

  EXPECT_EQ(expected_msg.name, _msg->name);
  EXPECT_EQ(expected_msg.view_controller, _msg->view_controller);
  EXPECT_EQ(expected_msg.projection_type, _msg->projection_type);
}

void createTestMsg(ros_gz_interfaces::msg::ParamVec & _msg)
{
  createTestMsg(_msg.header);

  rcl_interfaces::msg::Parameter p;
  p.name = "parameter_name_foo";
  p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  p.value.string_value = "parameter_value_foo";
  _msg.params.push_back(p);
}

void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::ParamVec> & _msg)
{
  ros_gz_interfaces::msg::ParamVec expected_msg;
  createTestMsg(expected_msg);
  compareTestMsg(_msg->header);

  EXPECT_EQ(expected_msg.params.size(), _msg->params.size());

  // Handle the case that the source was a Param_V message
  if (_msg->params[0].name.find("param_0") != std::string::npos) {
    EXPECT_EQ("param_0/" + expected_msg.params[0].name, _msg->params[0].name);
  } else {
    EXPECT_EQ(expected_msg.params[0].name, _msg->params[0].name);
  }
  EXPECT_EQ(expected_msg.params[0].value.type, _msg->params[0].value.type);
  EXPECT_EQ(expected_msg.params[0].value.string_value, _msg->params[0].value.string_value);
}

void createTestMsg(ros_gz_interfaces::msg::SensorNoise & _msg)
{
  createTestMsg(_msg.header);

  _msg.type = 3;
  _msg.mean = 100;
  _msg.stddev = 200;
  _msg.bias_mean = 300;
  _msg.bias_stddev = 400;
  _msg.precision = 500;
  _msg.dynamic_bias_stddev = 600;
}

void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::SensorNoise> & _msg)
{
  ros_gz_interfaces::msg::SensorNoise expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_EQ(expected_msg.mean, _msg->mean);
  EXPECT_EQ(expected_msg.stddev, _msg->stddev);
  EXPECT_EQ(expected_msg.bias_mean, _msg->bias_mean);
  EXPECT_EQ(expected_msg.bias_stddev, _msg->bias_stddev);
  EXPECT_EQ(expected_msg.precision, _msg->precision);
  EXPECT_EQ(expected_msg.dynamic_bias_stddev, _msg->dynamic_bias_stddev);
}

void createTestMsg(ros_gz_interfaces::msg::StringVec & _msg)
{
  createTestMsg(_msg.header);

  _msg.data.emplace_back("test_string_msg_v_data");
}

void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::StringVec> & _msg)
{
  ros_gz_interfaces::msg::StringVec expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);

  EXPECT_EQ(expected_msg.data, _msg->data);
}

void createTestMsg(ros_gz_interfaces::msg::TrackVisual & _msg)
{
  createTestMsg(_msg.header);
  createTestMsg(_msg.xyz);

  _msg.name = "test_track_visual";
  _msg.id = 15;
  _msg.inherit_orientation = true;
  _msg.min_dist = 1.1;
  _msg.max_dist = 1.5;
  _msg.is_static = true;
  _msg.use_model_frame = true;
  _msg.inherit_yaw = true;
}

void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::TrackVisual> & _msg)
{
  ros_gz_interfaces::msg::TrackVisual expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->xyz));

  EXPECT_EQ(expected_msg.name, _msg->name);
  EXPECT_EQ(expected_msg.id, _msg->id);
  EXPECT_EQ(expected_msg.inherit_orientation, _msg->inherit_orientation);
  EXPECT_EQ(expected_msg.min_dist, _msg->min_dist);
  EXPECT_EQ(expected_msg.max_dist, _msg->max_dist);
  EXPECT_EQ(expected_msg.is_static, _msg->is_static);
  EXPECT_EQ(expected_msg.use_model_frame, _msg->use_model_frame);
  EXPECT_EQ(expected_msg.inherit_yaw, _msg->inherit_yaw);
}

void createTestMsg(ros_gz_interfaces::msg::VideoRecord & _msg)
{
  createTestMsg(_msg.header);

  _msg.start = true;
  _msg.stop = true;
  _msg.format = "test_video_record_format";
  _msg.save_filename = "test_video_record_save_filename";
}

void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::VideoRecord> & _msg)
{
  ros_gz_interfaces::msg::VideoRecord expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);

  EXPECT_EQ(expected_msg.start, _msg->start);
  EXPECT_EQ(expected_msg.stop, _msg->stop);
  EXPECT_EQ(expected_msg.format, _msg->format);
  EXPECT_EQ(expected_msg.save_filename, _msg->save_filename);
}

void createTestMsg(ros_gz_interfaces::msg::JointWrench & _msg)
{
  createTestMsg(_msg.header);
  _msg.body_1_name.data = "body1";
  _msg.body_2_name.data = "body2";
  _msg.body_1_id.data = 1;
  _msg.body_2_id.data = 2;
  createTestMsg(_msg.body_1_wrench);
  createTestMsg(_msg.body_2_wrench);
}

void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::JointWrench> & _msg)
{
  ros_gz_interfaces::msg::JointWrench expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.body_1_name, _msg->body_1_name);
  EXPECT_EQ(expected_msg.body_2_name, _msg->body_2_name);
  EXPECT_EQ(expected_msg.body_1_id, _msg->body_1_id);
  EXPECT_EQ(expected_msg.body_2_id, _msg->body_2_id);

  compareTestMsg(_msg->header);
  compareTestMsg(std::make_shared<geometry_msgs::msg::Wrench>(_msg->body_1_wrench));
  compareTestMsg(std::make_shared<geometry_msgs::msg::Wrench>(_msg->body_2_wrench));
}

void createTestMsg(ros_gz_interfaces::msg::Altimeter & _msg)
{
  createTestMsg(_msg.header);
  _msg.vertical_position = 100;
  _msg.vertical_velocity = 200;
  _msg.vertical_reference = 300;
}

void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::Altimeter> & _msg)
{
  ros_gz_interfaces::msg::Altimeter expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.vertical_position, _msg->vertical_position);
  EXPECT_EQ(expected_msg.vertical_velocity, _msg->vertical_velocity);
  EXPECT_EQ(expected_msg.vertical_reference, _msg->vertical_reference);

  compareTestMsg(_msg->header);
}

void createTestMsg(ros_gz_interfaces::msg::Entity & _msg)
{
  _msg.id = 1;
  _msg.name = "entity";
  _msg.type = ros_gz_interfaces::msg::Entity::VISUAL;
}

void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::Entity> & _msg)
{
  ros_gz_interfaces::msg::Entity expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.id, _msg->id);
  EXPECT_EQ(expected_msg.name, _msg->name);
  EXPECT_EQ(expected_msg.type, _msg->type);
}

void createTestMsg(ros_gz_interfaces::msg::EntityWrench & _msg)
{
  createTestMsg(_msg.header);
  createTestMsg(_msg.entity);
  createTestMsg(_msg.wrench);
}

void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::EntityWrench> & _msg)
{
  ros_gz_interfaces::msg::EntityWrench expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<std_msgs::msg::Header>(_msg->header));
  compareTestMsg(std::make_shared<ros_gz_interfaces::msg::Entity>(_msg->entity));
  compareTestMsg(std::make_shared<geometry_msgs::msg::Wrench>(_msg->wrench));
}

void createTestMsg(ros_gz_interfaces::msg::Contact & _msg)
{
  createTestMsg(_msg.collision1);
  createTestMsg(_msg.collision2);

  geometry_msgs::msg::Vector3 vector_msg;
  createTestMsg(vector_msg);

  ros_gz_interfaces::msg::JointWrench joint_wrench_msg;
  createTestMsg(joint_wrench_msg);

  for (int i = 0; i < 10; i++) {
    _msg.depths.push_back(i);
    _msg.positions.push_back(vector_msg);
    _msg.normals.push_back(vector_msg);
    _msg.wrenches.push_back(joint_wrench_msg);
  }
}

void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::Contact> & _msg)
{
  ros_gz_interfaces::msg::Contact expected_msg;
  createTestMsg(expected_msg);
  compareTestMsg(std::make_shared<ros_gz_interfaces::msg::Entity>(_msg->collision1));
  compareTestMsg(std::make_shared<ros_gz_interfaces::msg::Entity>(_msg->collision2));
  EXPECT_EQ(expected_msg.depths.size(), _msg->depths.size());
  EXPECT_EQ(expected_msg.positions.size(), _msg->positions.size());
  EXPECT_EQ(expected_msg.normals.size(), _msg->normals.size());
  EXPECT_EQ(expected_msg.wrenches.size(), _msg->wrenches.size());
  for (size_t i = 0; i < _msg->depths.size(); i++) {
    EXPECT_EQ(expected_msg.depths.at(i), _msg->depths.at(i));
    compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->positions.at(i)));
    compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->normals.at(i)));
    compareTestMsg(std::make_shared<ros_gz_interfaces::msg::JointWrench>(_msg->wrenches.at(i)));
  }
}

void createTestMsg(ros_gz_interfaces::msg::Contacts & _msg)
{
  createTestMsg(_msg.header);

  ros_gz_interfaces::msg::Contact contact_msg;
  createTestMsg(contact_msg);

  for (int i = 0; i < 10; i++) {
    _msg.contacts.push_back(contact_msg);
  }
}

void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::Contacts> & _msg)
{
  ros_gz_interfaces::msg::Contacts expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_EQ(expected_msg.contacts.size(), _msg->contacts.size());
  for (size_t i = 0; i < _msg->contacts.size(); i++) {
    compareTestMsg(std::make_shared<ros_gz_interfaces::msg::Contact>(_msg->contacts.at(i)));
  }
}

<<<<<<< HEAD
#if HAVE_DATAFRAME
=======
>>>>>>> 63b651a (Garden EOL (#662))
void createTestMsg(ros_gz_interfaces::msg::Dataframe & _msg)
{
  createTestMsg(_msg.header);

  _msg.rssi = -10.3;
  _msg.src_address = "localhost:8080";
  _msg.dst_address = "localhost:8081";
  _msg.data.resize(150, '1');
}

void compareTestMsg(const std::shared_ptr<ros_gz_interfaces::msg::Dataframe> & _msg)
{
  ros_gz_interfaces::msg::Dataframe expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_EQ(expected_msg.src_address, _msg->src_address);
  EXPECT_EQ(expected_msg.dst_address, _msg->dst_address);
  EXPECT_EQ(expected_msg.rssi, _msg->rssi);

  ASSERT_EQ(expected_msg.data.size(), _msg->data.size());
  for (size_t ii = 0; ii < _msg->data.size(); ++ii) {
    EXPECT_EQ(expected_msg.data[ii], _msg->data[ii]);
  }
}
<<<<<<< HEAD
#endif  // HAVE_DATAFRAME
=======
>>>>>>> 63b651a (Garden EOL (#662))

void createTestMsg(nav_msgs::msg::Odometry & _msg)
{
  createTestMsg(_msg.header);
  createTestMsg(_msg.pose.pose);
  createTestMsg(_msg.twist.twist);
  for (int i = 0; i < 36; i++) {
    _msg.pose.covariance[i] = i;
    _msg.twist.covariance[i] = i;
  }
}

void compareTestMsg(const std::shared_ptr<nav_msgs::msg::Odometry> & _msg)
{
  compareTestMsg(_msg->header);
  compareTestMsg(_msg->pose.pose);
  compareTestMsg(std::make_shared<geometry_msgs::msg::Twist>(_msg->twist.twist));
}

void createTestMsg(sensor_msgs::msg::Image & _msg)
{
  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);

  _msg.header = header_msg;
  _msg.width = 320;
  _msg.height = 240;
  _msg.encoding = "rgb8";
  _msg.is_bigendian = false;
  _msg.step = _msg.width * 3;
  _msg.data.resize(_msg.height * _msg.step, '1');
}

void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::Image> & _msg)
{
  sensor_msgs::msg::Image expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_EQ(expected_msg.width, _msg->width);
  EXPECT_EQ(expected_msg.height, _msg->height);
  EXPECT_EQ(expected_msg.encoding, _msg->encoding);
  EXPECT_EQ(expected_msg.is_bigendian, _msg->is_bigendian);
  EXPECT_EQ(expected_msg.step, _msg->step);
}

void createTestMsg(sensor_msgs::msg::CameraInfo & _msg)
{
  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);

  _msg.header = header_msg;
  _msg.width = 320;
  _msg.height = 240;
  _msg.distortion_model = "plumb_bob";
  _msg.d.resize(5);
  _msg.d[0] = 1;
  _msg.d[1] = 2;
  _msg.d[2] = 3;
  _msg.d[3] = 4;
  _msg.d[4] = 5;

  _msg.k[0] = 1;
  _msg.k[1] = 0;
  _msg.k[2] = 0;
  _msg.k[3] = 0;
  _msg.k[4] = 1;
  _msg.k[5] = 0;
  _msg.k[6] = 0;
  _msg.k[7] = 0;
  _msg.k[8] = 1;

  _msg.r[0] = 1;
  _msg.r[1] = 0;
  _msg.r[2] = 0;
  _msg.r[3] = 0;
  _msg.r[4] = 1;
  _msg.r[5] = 0;
  _msg.r[6] = 0;
  _msg.r[7] = 0;
  _msg.r[8] = 1;

  _msg.p[0] = 1;
  _msg.p[1] = 0;
  _msg.p[2] = 0;
  _msg.p[3] = 0;
  _msg.p[4] = 0;
  _msg.p[5] = 1;
  _msg.p[6] = 0;
  _msg.p[7] = 0;
  _msg.p[8] = 0;
  _msg.p[9] = 0;
  _msg.p[10] = 1;
  _msg.p[11] = 0;
}

void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::CameraInfo> & _msg)
{
  sensor_msgs::msg::CameraInfo expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_EQ(expected_msg.width, _msg->width);
  EXPECT_EQ(expected_msg.height, _msg->height);
  EXPECT_EQ(expected_msg.distortion_model, _msg->distortion_model);

  for (auto i = 0; i < 12; ++i) {
    EXPECT_EQ(expected_msg.p[i], _msg->p[i]);

    if (i > 8) {
      continue;
    }

    EXPECT_EQ(expected_msg.k[i], _msg->k[i]);
    EXPECT_EQ(expected_msg.r[i], _msg->r[i]);

    if (i > 4) {
      continue;
    }

    EXPECT_EQ(expected_msg.d[i], _msg->d[i]);
  }
}

void createTestMsg(sensor_msgs::msg::FluidPressure & _msg)
{
  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);

  _msg.header = header_msg;
  _msg.fluid_pressure = 0.123;
  _msg.variance = 0.456;
}

void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::FluidPressure> & _msg)
{
  sensor_msgs::msg::FluidPressure expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_FLOAT_EQ(expected_msg.fluid_pressure, _msg->fluid_pressure);
  EXPECT_FLOAT_EQ(expected_msg.variance, _msg->variance);
}

void createTestMsg(sensor_msgs::msg::Imu & _msg)
{
  std_msgs::msg::Header header_msg;
  geometry_msgs::msg::Quaternion quaternion_msg;
  geometry_msgs::msg::Vector3 vector3_msg;

  createTestMsg(header_msg);
  createTestMsg(quaternion_msg);
  createTestMsg(vector3_msg);

  _msg.header = header_msg;
  _msg.orientation = quaternion_msg;
  _msg.angular_velocity = vector3_msg;
  _msg.linear_acceleration = vector3_msg;
#ifdef GZ_MSGS_IMU_HAS_COVARIANCE
  _msg.orientation_covariance = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  _msg.angular_velocity_covariance = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  _msg.linear_acceleration_covariance = {1, 2, 3, 4, 5, 6, 7, 8, 9};
#endif
}

void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::Imu> & _msg)
{
  compareTestMsg(_msg->header);
  compareTestMsg(_msg->orientation);
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->angular_velocity));
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->linear_acceleration));

#ifdef GZ_MSGS_IMU_HAS_COVARIANCE
  for (int i = 0; i < 9; ++i) {
    EXPECT_EQ(_msg->orientation_covariance[i], i + 1);
    EXPECT_EQ(_msg->angular_velocity_covariance[i], i + 1);
    EXPECT_EQ(_msg->linear_acceleration_covariance[i], i + 1);
  }
#endif
}

void createTestMsg(sensor_msgs::msg::JointState & _msg)
{
  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);

  _msg.header = header_msg;
  _msg.name = {"joint_0", "joint_1", "joint_2"};
  _msg.position = {1, 1, 1};
  _msg.velocity = {2, 2, 2};
  _msg.effort = {3, 3, 3};
}

void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::JointState> & _msg)
{
  sensor_msgs::msg::JointState expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);

  ASSERT_EQ(expected_msg.name.size(), _msg->name.size());
  ASSERT_EQ(expected_msg.position.size(), _msg->position.size());
  ASSERT_EQ(expected_msg.velocity.size(), _msg->velocity.size());
  ASSERT_EQ(expected_msg.effort.size(), _msg->effort.size());

  for (auto i = 0u; i < _msg->position.size(); ++i) {
    EXPECT_EQ(expected_msg.name[i], _msg->name[i]);
    EXPECT_FLOAT_EQ(expected_msg.position[i], _msg->position[i]);
    EXPECT_FLOAT_EQ(expected_msg.velocity[i], _msg->velocity[i]);
    EXPECT_FLOAT_EQ(expected_msg.effort[i], _msg->effort[i]);
  }
}

void createTestMsg(sensor_msgs::msg::Joy & _msg)
{
  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);

  _msg.header = header_msg;
  _msg.axes = {0.5, 0.5, 0.5};
  _msg.buttons = {0, 0, 0};
}

void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::Joy> & _msg)
{
  sensor_msgs::msg::Joy expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);

  for (auto i = 0u; i < _msg->axes.size(); ++i) {
    EXPECT_FLOAT_EQ(expected_msg.buttons[i], _msg->buttons[i]);
  }

  for (auto i = 0u; i < _msg->buttons.size(); ++i) {
    EXPECT_EQ(expected_msg.buttons[i], _msg->buttons[i]);
  }
}

void createTestMsg(sensor_msgs::msg::LaserScan & _msg)
{
  const unsigned int num_readings = 100u;

  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);

  _msg.header = header_msg;
  _msg.angle_min = -1.57;
  _msg.angle_max = 1.57;
  _msg.angle_increment = 3.14 / num_readings;
  _msg.scan_time = 0;
  _msg.range_min = 1;
  _msg.range_max = 2;
  _msg.ranges.resize(num_readings, 0);
  _msg.intensities.resize(num_readings, 1);
}

void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::LaserScan> & _msg)
{
  sensor_msgs::msg::LaserScan expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_FLOAT_EQ(expected_msg.angle_min, _msg->angle_min);
  EXPECT_FLOAT_EQ(expected_msg.angle_max, _msg->angle_max);
  EXPECT_FLOAT_EQ(expected_msg.angle_increment, _msg->angle_increment);
  EXPECT_FLOAT_EQ(0, _msg->scan_time);
  EXPECT_FLOAT_EQ(expected_msg.range_min, _msg->range_min);
  EXPECT_FLOAT_EQ(expected_msg.range_max, _msg->range_max);

  const unsigned int num_readings =
    (_msg->angle_max - _msg->angle_min) / _msg->angle_increment;
  for (auto i = 0u; i < num_readings; ++i) {
    EXPECT_FLOAT_EQ(expected_msg.ranges[i], _msg->ranges[i]);
    EXPECT_FLOAT_EQ(expected_msg.intensities[i], _msg->intensities[i]);
  }
}

void createTestMsg(sensor_msgs::msg::MagneticField & _msg)
{
  std_msgs::msg::Header header_msg;
  geometry_msgs::msg::Vector3 vector3_msg;

  createTestMsg(header_msg);
  createTestMsg(vector3_msg);

  _msg.header = header_msg;
  _msg.magnetic_field = vector3_msg;
}

void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::MagneticField> & _msg)
{
  compareTestMsg(_msg->header);
  compareTestMsg(std::make_shared<geometry_msgs::msg::Vector3>(_msg->magnetic_field));
}

void createTestMsg(sensor_msgs::msg::NavSatFix & _msg)
{
  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);

  _msg.header = header_msg;
  _msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  _msg.latitude = 0.00;
  _msg.longitude = 0.00;
  _msg.altitude = 0.00;
  _msg.position_covariance = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  _msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
}

void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::NavSatFix> & _msg)
{
  sensor_msgs::msg::NavSatFix expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_EQ(expected_msg.status, _msg->status);
  EXPECT_FLOAT_EQ(expected_msg.latitude, _msg->latitude);
  EXPECT_FLOAT_EQ(expected_msg.longitude, _msg->longitude);
  EXPECT_FLOAT_EQ(expected_msg.altitude, _msg->altitude);
  EXPECT_EQ(expected_msg.position_covariance_type, _msg->position_covariance_type);

  for (auto i = 0u; i < 9; ++i) {
    EXPECT_FLOAT_EQ(0, _msg->position_covariance[i]);
  }
}

void createTestMsg(sensor_msgs::msg::PointCloud2 & _msg)
{
  createTestMsg(_msg.header);

  sensor_msgs::msg::PointField field;
  field.name = "x";
  field.offset = 0;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  _msg.fields.push_back(field);

  uint32_t height = 4;
  uint32_t width = 2;

  _msg.height = height;
  _msg.width = width;
  _msg.is_bigendian = false;
  _msg.point_step = 4;
  _msg.row_step = 4 * width;
  _msg.is_dense = true;

  _msg.data.resize(_msg.row_step * _msg.height);
  uint8_t * msgBufferIndex = _msg.data.data();

  for (uint32_t j = 0; j < height; ++j) {
    for (uint32_t i = 0; i < width; ++i) {
      *reinterpret_cast<float *>(msgBufferIndex + _msg.fields[0].offset) =
        j * width + i;
      msgBufferIndex += _msg.point_step;
    }
  }
}

void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::PointCloud2> & _msg)
{
  compareTestMsg(_msg->header);

  uint32_t height = 4;
  uint32_t width = 2;

  EXPECT_EQ(height, _msg->height);
  EXPECT_EQ(width, _msg->width);
  EXPECT_FALSE(_msg->is_bigendian);
  EXPECT_EQ(4u, _msg->point_step);
  EXPECT_EQ(4U * width, _msg->row_step);
  EXPECT_TRUE(_msg->is_dense);

  unsigned char * msgBufferIndex =
    const_cast<unsigned char *>(_msg->data.data());

  for (uint32_t j = 0; j < height; ++j) {
    for (uint32_t i = 0; i < width; ++i) {
      float * value =
        reinterpret_cast<float *>(msgBufferIndex + _msg->fields[0].offset);

      EXPECT_FLOAT_EQ(static_cast<float>(j * width + i), *value);
      msgBufferIndex += _msg->point_step;
    }
  }
}

void createTestMsg(sensor_msgs::msg::BatteryState & _msg)
{
  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);

  _msg.header = header_msg;
  _msg.voltage = 123;
  _msg.current = 456;
  _msg.charge = 789;
  _msg.capacity = 321;
  _msg.percentage = 654;
  _msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
}

void compareTestMsg(const std::shared_ptr<sensor_msgs::msg::BatteryState> & _msg)
{
  sensor_msgs::msg::BatteryState expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_EQ(expected_msg.voltage, _msg->voltage);
  EXPECT_EQ(expected_msg.current, _msg->current);
  EXPECT_EQ(expected_msg.charge, _msg->charge);
  EXPECT_EQ(expected_msg.capacity, _msg->capacity);
  EXPECT_EQ(expected_msg.percentage, _msg->percentage);
  EXPECT_EQ(expected_msg.power_supply_status, _msg->power_supply_status);
}

void createTestMsg(trajectory_msgs::msg::JointTrajectoryPoint & _msg)
{
  const auto number_of_joints = 7;

  for (auto i = 0; i < number_of_joints; ++i) {
    _msg.positions.push_back(1.1 * i);
    _msg.velocities.push_back(2.2 * i);
    _msg.accelerations.push_back(3.3 * i);
    _msg.effort.push_back(4.4 * i);
  }
  _msg.time_from_start.sec = 12345;
  _msg.time_from_start.nanosec = 67890;
}

void compareTestMsg(const std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint> & _msg)
{
  trajectory_msgs::msg::JointTrajectoryPoint expected_msg;
  createTestMsg(expected_msg);

  for (auto i = 0u; i < _msg->positions.size(); ++i) {
    EXPECT_EQ(expected_msg.positions[i], _msg->positions[i]);
  }

  for (auto i = 0u; i < _msg->velocities.size(); ++i) {
    EXPECT_EQ(expected_msg.velocities[i], _msg->velocities[i]);
  }

  for (auto i = 0u; i < _msg->accelerations.size(); ++i) {
    EXPECT_EQ(expected_msg.accelerations[i], _msg->accelerations[i]);
  }

  for (auto i = 0u; i < _msg->effort.size(); ++i) {
    EXPECT_EQ(expected_msg.effort[i], _msg->effort[i]);
  }

  EXPECT_EQ(expected_msg.time_from_start.sec, _msg->time_from_start.sec);
  EXPECT_EQ(expected_msg.time_from_start.nanosec, _msg->time_from_start.nanosec);
}

void createTestMsg(trajectory_msgs::msg::JointTrajectory & _msg)
{
  const auto number_of_joints = 7;
  const auto number_of_trajectory_points = 10;

  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);
  _msg.header = header_msg;

  for (auto i = 0; i < number_of_joints; ++i) {
    _msg.joint_names.push_back("joint_" + std::to_string(i));
  }

  for (auto j = 0; j < number_of_trajectory_points; ++j) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    createTestMsg(point);
    _msg.points.push_back(point);
  }
}

void compareTestMsg(const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & _msg)
{
  trajectory_msgs::msg::JointTrajectory expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);

  for (auto i = 0u; i < _msg->joint_names.size(); ++i) {
    EXPECT_EQ(expected_msg.joint_names[i], _msg->joint_names[i]);
  }

  for (auto i = 0u; i < _msg->points.size(); ++i) {
    compareTestMsg(std::make_shared<trajectory_msgs::msg::JointTrajectoryPoint>(_msg->points[i]));
  }
}

void createTestMsg(rcl_interfaces::msg::ParameterValue & _msg)
{
  _msg.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  _msg.string_value = "foobar";
}

void compareTestMsg(const std::shared_ptr<rcl_interfaces::msg::ParameterValue> & _msg)
{
  rcl_interfaces::msg::ParameterValue expected_msg;
  createTestMsg(expected_msg);
  EXPECT_EQ(expected_msg.type, _msg->type);
  EXPECT_EQ(expected_msg.string_value, _msg->string_value);
}

void createTestMsg(vision_msgs::msg::Detection2D & _msg)
{
  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);
  _msg.header = header_msg;

  vision_msgs::msg::ObjectHypothesisWithPose class_prob;
  class_prob.hypothesis.class_id = "1";
  class_prob.hypothesis.score = 1.0;
  _msg.results.push_back(class_prob);

  _msg.bbox.size_x = 2.0;
  _msg.bbox.size_y = 4.0;
  _msg.bbox.center.position.x = 3.0;
  _msg.bbox.center.position.y = 4.0;
}

void compareTestMsg(const std::shared_ptr<vision_msgs::msg::Detection2D> & _msg)
{
  vision_msgs::msg::Detection2D expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_EQ(expected_msg.results.size(), _msg->results.size());
  for (size_t i = 0; i < _msg->results.size(); i++) {
    EXPECT_EQ(expected_msg.results[i].hypothesis.class_id, _msg->results[i].hypothesis.class_id);
    EXPECT_EQ(expected_msg.results[i].hypothesis.score, _msg->results[i].hypothesis.score);
  }
  EXPECT_EQ(expected_msg.bbox.size_x, _msg->bbox.size_x);
  EXPECT_EQ(expected_msg.bbox.size_y, _msg->bbox.size_y);
  EXPECT_EQ(expected_msg.bbox.center.position.x, _msg->bbox.center.position.x);
  EXPECT_EQ(expected_msg.bbox.center.position.y, _msg->bbox.center.position.y);
}

void createTestMsg(vision_msgs::msg::Detection2DArray & _msg)
{
  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);
  _msg.header = header_msg;

  for (size_t i = 0; i < 4; i++) {
    vision_msgs::msg::Detection2D detection;
    createTestMsg(detection);
    _msg.detections.push_back(detection);
  }
}

void compareTestMsg(const std::shared_ptr<vision_msgs::msg::Detection2DArray> & _msg)
{
  vision_msgs::msg::Detection2DArray expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_EQ(expected_msg.detections.size(), _msg->detections.size());
  for (size_t i = 0; i < _msg->detections.size(); i++) {
    compareTestMsg(std::make_shared<vision_msgs::msg::Detection2D>(_msg->detections[i]));
  }
}

void createTestMsg(vision_msgs::msg::Detection3D & _msg)
{
  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);
  _msg.header = header_msg;

  vision_msgs::msg::ObjectHypothesisWithPose class_prob;
  class_prob.hypothesis.class_id = "1";
  class_prob.hypothesis.score = 1.0;
  _msg.results.push_back(class_prob);

  _msg.bbox.size.x = 3.0;
  _msg.bbox.size.y = 5.0;
  _msg.bbox.size.z = 7.0;
  _msg.bbox.center.position.x = 2.0;
  _msg.bbox.center.position.y = 4.0;
  _msg.bbox.center.position.z = 6.0;
  _msg.bbox.center.orientation.x = 0.733;
  _msg.bbox.center.orientation.y = 0.462;
  _msg.bbox.center.orientation.z = 0.191;
  _msg.bbox.center.orientation.w = 0.462;
}

void compareTestMsg(const std::shared_ptr<vision_msgs::msg::Detection3D> & _msg)
{
  vision_msgs::msg::Detection3D expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_EQ(expected_msg.results.size(), _msg->results.size());
  for (size_t i = 0; i < _msg->results.size(); i++) {
    EXPECT_EQ(expected_msg.results[i].hypothesis.class_id, _msg->results[i].hypothesis.class_id);
    EXPECT_EQ(expected_msg.results[i].hypothesis.score, _msg->results[i].hypothesis.score);
  }
  EXPECT_EQ(expected_msg.bbox.size.x, _msg->bbox.size.x);
  EXPECT_EQ(expected_msg.bbox.size.y, _msg->bbox.size.y);
  EXPECT_EQ(expected_msg.bbox.size.z, _msg->bbox.size.z);
  EXPECT_EQ(expected_msg.bbox.center.position.x, _msg->bbox.center.position.x);
  EXPECT_EQ(expected_msg.bbox.center.position.y, _msg->bbox.center.position.y);
  EXPECT_EQ(expected_msg.bbox.center.position.z, _msg->bbox.center.position.z);
  EXPECT_EQ(expected_msg.bbox.center.orientation.x, _msg->bbox.center.orientation.x);
  EXPECT_EQ(expected_msg.bbox.center.orientation.y, _msg->bbox.center.orientation.y);
  EXPECT_EQ(expected_msg.bbox.center.orientation.z, _msg->bbox.center.orientation.z);
  EXPECT_EQ(expected_msg.bbox.center.orientation.w, _msg->bbox.center.orientation.w);
}

void createTestMsg(vision_msgs::msg::Detection3DArray & _msg)
{
  std_msgs::msg::Header header_msg;
  createTestMsg(header_msg);
  _msg.header = header_msg;

  for (size_t i = 0; i < 4; i++) {
    vision_msgs::msg::Detection3D detection;
    createTestMsg(detection);
    _msg.detections.push_back(detection);
  }
}

void compareTestMsg(const std::shared_ptr<vision_msgs::msg::Detection3DArray> & _msg)
{
  vision_msgs::msg::Detection3DArray expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(_msg->header);
  EXPECT_EQ(expected_msg.detections.size(), _msg->detections.size());
  for (size_t i = 0; i < _msg->detections.size(); i++) {
    compareTestMsg(std::make_shared<vision_msgs::msg::Detection3D>(_msg->detections[i]));
  }
}

}  // namespace testing
}  // namespace ros_gz_bridge
