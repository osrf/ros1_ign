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

#include "gz_test_msg.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <cstddef>

#if GZ_MSGS_MAJOR_VERSION >= 10
#define GZ_MSGS_IMU_HAS_COVARIANCE
#endif

namespace ros_gz_bridge
{
namespace testing
{

void createTestMsg(gz::msgs::Any & _msg)
{
  _msg.set_type(gz::msgs::Any_ValueType::Any_ValueType_STRING);
  _msg.set_string_value("foobar");
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Any> & _msg)
{
  gz::msgs::Any expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.type(), _msg->type());
  EXPECT_EQ(expected_msg.string_value(), _msg->string_value());
}

void createTestMsg(gz::msgs::Boolean & _msg)
{
  _msg.set_data(true);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Boolean> & _msg)
{
  gz::msgs::Boolean expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.data(), _msg->data());
}

void createTestMsg(gz::msgs::Color & _msg)
{
  _msg.set_r(0.2);
  _msg.set_g(0.4);
  _msg.set_b(0.6);
  _msg.set_a(0.8);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Color> & _msg)
{
  gz::msgs::Color expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.r(), _msg->r());
  EXPECT_EQ(expected_msg.g(), _msg->g());
  EXPECT_EQ(expected_msg.b(), _msg->b());
  EXPECT_EQ(expected_msg.a(), _msg->a());
}

void createTestMsg(gz::msgs::Empty &)
{
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Empty> &)
{
}

void createTestMsg(gz::msgs::Float & _msg)
{
  _msg.set_data(1.5);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Float> & _msg)
{
  gz::msgs::Float expected_msg;
  createTestMsg(expected_msg);

  EXPECT_FLOAT_EQ(expected_msg.data(), _msg->data());
}

void createTestMsg(gz::msgs::Float_V & _msg)
{
  gz::msgs::Float msg;
  createTestMsg(msg);
  _msg.add_data(msg.data());
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Float_V> & _msg)
{
  gz::msgs::Float_V expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.data().size(), _msg->data().size());
  EXPECT_EQ(1, _msg->data().size());
  EXPECT_FLOAT_EQ(expected_msg.data()[0], _msg->data()[0]);
}

void createTestMsg(gz::msgs::Double & _msg)
{
  _msg.set_data(1.5);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Double> & _msg)
{
  gz::msgs::Double expected_msg;
  createTestMsg(expected_msg);

  EXPECT_DOUBLE_EQ(expected_msg.data(), _msg->data());
}

void createTestMsg(gz::msgs::Int32 & _msg)
{
  _msg.set_data(-10);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Int32> & _msg)
{
  gz::msgs::Int32 expected_msg;
  createTestMsg(expected_msg);

  EXPECT_DOUBLE_EQ(expected_msg.data(), _msg->data());
}

void createTestMsg(gz::msgs::UInt32 & _msg)
{
  _msg.set_data(1);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::UInt32> & _msg)
{
  gz::msgs::UInt32 expected_msg;
  createTestMsg(expected_msg);

  EXPECT_DOUBLE_EQ(expected_msg.data(), _msg->data());
}

void createTestMsg(gz::msgs::Header & _msg)
{
  auto seq_entry = _msg.add_data();
  seq_entry->set_key("seq");
  seq_entry->add_value("1");
  _msg.mutable_stamp()->set_sec(2);
  _msg.mutable_stamp()->set_nsec(3);
  auto frame_id_entry = _msg.add_data();
  frame_id_entry->set_key("frame_id");
  frame_id_entry->add_value("frame_id_value");
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Header> & _msg)
{
  // TODO(anyone): Review this
  gz::msgs::Header expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.stamp().sec(), _msg->stamp().sec());
  EXPECT_EQ(expected_msg.stamp().nsec(), _msg->stamp().nsec());
  // EXPECT_GE(_msg->data_size(), 2);
  // EXPECT_EQ(expected_msg.data(0).key(), "seq");
  // EXPECT_EQ(1, _msg->data(0).value_size());
  // std::string value = _msg->data(0).value(0);
  // try {
  //   uint32_t ul = std::stoul(value, nullptr);
  //   EXPECT_GE(ul, 0u);
  // } catch (std::exception & e) {
  //   FAIL();
  // }
  // EXPECT_EQ(expected_msg.data(1).key(), _msg->data(1).key());
  // EXPECT_EQ(1, _msg->data(1).value_size());
  // EXPECT_EQ(expected_msg.data(1).value(0), _msg->data(1).value(0));
}

void createTestMsg(gz::msgs::Clock & _msg)
{
  _msg.mutable_sim()->set_sec(1);
  _msg.mutable_sim()->set_nsec(2);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Clock> & _msg)
{
  gz::msgs::Clock expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.sim().sec(), _msg->sim().sec());
  EXPECT_EQ(expected_msg.sim().nsec(), _msg->sim().nsec());
}

void createTestMsg(gz::msgs::StringMsg & _msg)
{
  _msg.set_data("string");
}

void compareTestMsg(const std::shared_ptr<gz::msgs::StringMsg> & _msg)
{
  gz::msgs::StringMsg expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.data(), _msg->data());
}

void createTestMsg(gz::msgs::Quaternion & _msg)
{
  _msg.set_x(1.0);
  _msg.set_y(2.0);
  _msg.set_z(3.0);
  _msg.set_w(4.0);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Quaternion> & _msg)
{
  gz::msgs::Quaternion expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.x(), _msg->x());
  EXPECT_EQ(expected_msg.y(), _msg->y());
  EXPECT_EQ(expected_msg.z(), _msg->z());
  EXPECT_EQ(expected_msg.w(), _msg->w());
}

void createTestMsg(gz::msgs::Vector3d & _msg)
{
  _msg.set_x(1.0);
  _msg.set_y(2.0);
  _msg.set_z(3.0);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Vector3d> & _msg)
{
  gz::msgs::Vector3d expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.x(), _msg->x());
  EXPECT_EQ(expected_msg.y(), _msg->y());
  EXPECT_EQ(expected_msg.z(), _msg->z());
}

void createTestMsg(gz::msgs::Altimeter & _msg)
{
  createTestMsg(*_msg.mutable_header());
  _msg.set_vertical_position(100);
  _msg.set_vertical_velocity(200);
  _msg.set_vertical_reference(300);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Altimeter> & _msg)
{
  gz::msgs::Altimeter expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.vertical_position(), _msg->vertical_position());
  EXPECT_EQ(expected_msg.vertical_velocity(), _msg->vertical_velocity());
  EXPECT_EQ(expected_msg.vertical_reference(), _msg->vertical_reference());
  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));
}

void createTestMsg(gz::msgs::SensorNoise & _msg)
{
  createTestMsg(*_msg.mutable_header());
  _msg.set_type(gz::msgs::SensorNoise_Type::SensorNoise_Type_GAUSSIAN_QUANTIZED);
  _msg.set_mean(100);
  _msg.set_stddev(200);
  _msg.set_bias_mean(300);
  _msg.set_bias_stddev(400);
  _msg.set_precision(500);
  _msg.set_dynamic_bias_stddev(600);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::SensorNoise> & _msg)
{
  gz::msgs::SensorNoise expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(
    expected_msg.type(),
    gz::msgs::SensorNoise_Type::SensorNoise_Type_GAUSSIAN_QUANTIZED);
  EXPECT_EQ(expected_msg.mean(), _msg->mean());
  EXPECT_EQ(expected_msg.stddev(), _msg->stddev());
  EXPECT_EQ(expected_msg.bias_mean(), _msg->bias_mean());
  EXPECT_EQ(expected_msg.bias_stddev(), _msg->bias_stddev());
  EXPECT_EQ(expected_msg.precision(), _msg->precision());
  EXPECT_EQ(expected_msg.dynamic_bias_stddev(), _msg->dynamic_bias_stddev());
  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));
}

void createTestMsg(gz::msgs::Param & _msg)
{
  createTestMsg(*_msg.mutable_header());
  auto * params = _msg.mutable_params();
  {
    gz::msgs::Any param;
    param.set_type(gz::msgs::Any_ValueType::Any_ValueType_STRING);
    param.set_string_value("parameter_value_foo");
    (*params)["parameter_name_foo"] = param;
  }
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Param> & _msg)
{
  gz::msgs::Param expected_msg;
  createTestMsg(expected_msg);
  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));
  EXPECT_EQ(expected_msg.params().size(), _msg->params().size());
}

void createTestMsg(gz::msgs::Param_V & _msg)
{
  createTestMsg(*_msg.mutable_header());
  auto param = _msg.mutable_param()->Add();
  createTestMsg(*param);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Param_V> & _msg)
{
  gz::msgs::Param_V expected_msg;
  createTestMsg(expected_msg);
  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));
  compareTestMsg(std::make_shared<gz::msgs::Param>(_msg->param().Get(0)));
}

void createTestMsg(gz::msgs::Pose & _msg)
{
  createTestMsg(*_msg.mutable_header());
  auto child_frame_id_entry = _msg.mutable_header()->add_data();
  child_frame_id_entry->set_key("child_frame_id");
  child_frame_id_entry->add_value("child_frame_id_value");

  createTestMsg(*_msg.mutable_position());
  createTestMsg(*_msg.mutable_orientation());
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Pose> & _msg)
{
  if (_msg->header().data_size() > 0) {
    compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));

    gz::msgs::Pose expected_msg;
    createTestMsg(expected_msg);

    if (_msg->header().data_size() > 2) {
      // child_frame_id
      ASSERT_EQ(3, expected_msg.header().data_size());
      ASSERT_EQ(3, _msg->header().data_size());
      EXPECT_EQ(
        expected_msg.header().data(2).key(),
        _msg->header().data(2).key());
      EXPECT_EQ(1, _msg->header().data(2).value_size());
      EXPECT_EQ(
        expected_msg.header().data(2).value(0),
        _msg->header().data(2).value(0));
    }
  }

  compareTestMsg(std::make_shared<gz::msgs::Vector3d>(_msg->position()));
  compareTestMsg(std::make_shared<gz::msgs::Quaternion>(_msg->orientation()));
}

void createTestMsg(gz::msgs::PoseWithCovariance & _msg)
{
  createTestMsg(*_msg.mutable_pose()->mutable_position());
  createTestMsg(*_msg.mutable_pose()->mutable_orientation());
<<<<<<< HEAD
=======
  createTestMsg(*_msg.mutable_pose()->mutable_header());
>>>>>>> 63b651a (Garden EOL (#662))
  for (int i = 0; i < 36; i++) {
    _msg.mutable_covariance()->add_data(i);
  }
}

void compareTestMsg(const std::shared_ptr<gz::msgs::PoseWithCovariance> & _msg)
{
  compareTestMsg(std::make_shared<gz::msgs::Vector3d>(_msg->pose().position()));
  compareTestMsg(std::make_shared<gz::msgs::Quaternion>(_msg->pose().orientation()));
  for (int i = 0; i < 36; i++) {
    EXPECT_EQ(_msg->covariance().data(i), i);
  }
}

void createTestMsg(gz::msgs::Pose_V & _msg)
{
  createTestMsg(*(_msg.mutable_header()));
  createTestMsg(*(_msg.add_pose()));
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Pose_V> & _msg)
{
  gz::msgs::Pose_V expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));
  compareTestMsg(std::make_shared<gz::msgs::Pose>(_msg->pose(0)));
}

void createTestMsg(gz::msgs::Twist & _msg)
{
  gz::msgs::Header header_msg;
  gz::msgs::Vector3d linear_msg;
  gz::msgs::Vector3d angular_msg;

  createTestMsg(header_msg);
  createTestMsg(linear_msg);
  createTestMsg(angular_msg);

  _msg.mutable_header()->CopyFrom(header_msg);
  _msg.mutable_linear()->CopyFrom(linear_msg);
  _msg.mutable_angular()->CopyFrom(angular_msg);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Twist> & _msg)
{
  if (_msg->has_header()) {
    compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));
  }
  compareTestMsg(std::make_shared<gz::msgs::Vector3d>(_msg->linear()));
  compareTestMsg(std::make_shared<gz::msgs::Vector3d>(_msg->angular()));
}

void createTestMsg(gz::msgs::TwistWithCovariance & _msg)
{
  gz::msgs::Vector3d linear_msg;
  gz::msgs::Vector3d angular_msg;
  gz::msgs::Header header_msg;

  createTestMsg(linear_msg);
  createTestMsg(angular_msg);
  createTestMsg(header_msg);

  _msg.mutable_twist()->mutable_linear()->CopyFrom(linear_msg);
  _msg.mutable_twist()->mutable_angular()->CopyFrom(angular_msg);
  _msg.mutable_twist()->mutable_header()->CopyFrom(header_msg);
  for (int i = 0; i < 36; i++) {
    _msg.mutable_covariance()->add_data(i);
  }
}

void compareTestMsg(const std::shared_ptr<gz::msgs::TwistWithCovariance> & _msg)
{
  compareTestMsg(std::make_shared<gz::msgs::Vector3d>(_msg->twist().linear()));
  compareTestMsg(std::make_shared<gz::msgs::Vector3d>(_msg->twist().angular()));
  for (int i = 0; i < 36; i++) {
    EXPECT_EQ(_msg->covariance().data()[i], i);
  }
}

void createTestMsg(gz::msgs::Wrench & _msg)
{
  gz::msgs::Header header_msg;
  gz::msgs::Vector3d force_msg;
  gz::msgs::Vector3d torque_msg;

  createTestMsg(header_msg);
  createTestMsg(force_msg);
  createTestMsg(torque_msg);

  _msg.mutable_header()->CopyFrom(header_msg);
  _msg.mutable_force()->CopyFrom(force_msg);
  _msg.mutable_torque()->CopyFrom(torque_msg);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Wrench> & _msg)
{
  compareTestMsg(std::make_shared<gz::msgs::Vector3d>(_msg->force()));
  compareTestMsg(std::make_shared<gz::msgs::Vector3d>(_msg->torque()));
}

void createTestMsg(gz::msgs::JointWrench & _msg)
{
  gz::msgs::Header header_msg;
  gz::msgs::Wrench body_1_wrench_msg;
  gz::msgs::Wrench body_2_wrench_msg;

  createTestMsg(header_msg);
  createTestMsg(body_1_wrench_msg);
  createTestMsg(body_2_wrench_msg);

  _msg.mutable_header()->CopyFrom(header_msg);
  _msg.set_body_1_name("body1");
  _msg.set_body_2_name("body2");
  _msg.set_body_1_id(1);
  _msg.set_body_2_id(2);
  _msg.mutable_body_1_wrench()->CopyFrom(body_1_wrench_msg);
  _msg.mutable_body_2_wrench()->CopyFrom(body_2_wrench_msg);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::JointWrench> & _msg)
{
  gz::msgs::JointWrench expected_msg;
  createTestMsg(expected_msg);
  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));
  EXPECT_EQ(expected_msg.body_1_name(), _msg->body_1_name());
  EXPECT_EQ(expected_msg.body_2_name(), _msg->body_2_name());
  EXPECT_EQ(expected_msg.body_1_id(), _msg->body_1_id());
  EXPECT_EQ(expected_msg.body_2_id(), _msg->body_2_id());
  compareTestMsg(std::make_shared<gz::msgs::Wrench>(_msg->body_1_wrench()));
  compareTestMsg(std::make_shared<gz::msgs::Wrench>(_msg->body_2_wrench()));
}

void createTestMsg(gz::msgs::Entity & _msg)
{
  _msg.set_id(1);
  _msg.set_name("entity");
  _msg.set_type(gz::msgs::Entity::VISUAL);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Entity> & _msg)
{
  gz::msgs::Entity expected_msg;
  createTestMsg(expected_msg);
  EXPECT_EQ(expected_msg.id(), _msg->id());
  EXPECT_EQ(expected_msg.name(), _msg->name());
  EXPECT_EQ(expected_msg.type(), _msg->type());
}

void createTestMsg(gz::msgs::EntityWrench & _msg)
{
  gz::msgs::Header header_msg;
  gz::msgs::Entity entity_msg;
  gz::msgs::Wrench wrench_msg;

  createTestMsg(header_msg);
  createTestMsg(entity_msg);
  createTestMsg(wrench_msg);

  _msg.mutable_header()->CopyFrom(header_msg);
  _msg.mutable_entity()->CopyFrom(entity_msg);
  _msg.mutable_wrench()->CopyFrom(wrench_msg);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::EntityWrench> & _msg)
{
  gz::msgs::EntityWrench expected_msg;
  createTestMsg(expected_msg);
  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));
  compareTestMsg(std::make_shared<gz::msgs::Entity>(_msg->entity()));
  compareTestMsg(std::make_shared<gz::msgs::Wrench>(_msg->wrench()));
}

void createTestMsg(gz::msgs::Contact & _msg)
{
  gz::msgs::Entity collision1;
  gz::msgs::Entity collision2;
  gz::msgs::Vector3d position_msg;
  gz::msgs::Vector3d normal_msg;
  gz::msgs::JointWrench wrench_msg;

  createTestMsg(collision1);
  createTestMsg(collision2);
  createTestMsg(position_msg);
  createTestMsg(normal_msg);
  createTestMsg(wrench_msg);

  _msg.clear_position();
  _msg.clear_normal();
  _msg.clear_wrench();

  for (int i = 0; i < 10; i++) {
    _msg.add_depth(i);
    auto position = _msg.add_position();
    position->set_x(position_msg.x());
    position->set_y(position_msg.y());
    position->set_z(position_msg.z());
    auto normal = _msg.add_normal();
    normal->set_x(normal_msg.x());
    normal->set_y(normal_msg.y());
    normal->set_z(normal_msg.z());
    auto wrench = _msg.add_wrench();
    wrench->mutable_header()->CopyFrom(wrench_msg.header());
    wrench->set_body_1_name(wrench_msg.body_1_name());
    wrench->set_body_2_name(wrench_msg.body_2_name());
    wrench->set_body_1_id(wrench_msg.body_1_id());
    wrench->set_body_2_id(wrench_msg.body_2_id());
    wrench->mutable_body_1_wrench()->CopyFrom(wrench_msg.body_1_wrench());
    wrench->mutable_body_2_wrench()->CopyFrom(wrench_msg.body_2_wrench());
  }
  _msg.mutable_collision1()->CopyFrom(collision1);
  _msg.mutable_collision2()->CopyFrom(collision2);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Contact> & _msg)
{
  gz::msgs::Contact expected_msg;
  createTestMsg(expected_msg);
  compareTestMsg(std::make_shared<gz::msgs::Entity>(_msg->collision1()));
  compareTestMsg(std::make_shared<gz::msgs::Entity>(_msg->collision2()));
  EXPECT_EQ(expected_msg.depth_size(), _msg->depth_size());
  EXPECT_EQ(expected_msg.position_size(), _msg->position_size());
  EXPECT_EQ(expected_msg.normal_size(), _msg->normal_size());
  EXPECT_EQ(expected_msg.wrench_size(), _msg->wrench_size());
  for (int i = 0; i < expected_msg.depth_size(); i++) {
    EXPECT_EQ(expected_msg.depth(i), _msg->depth(i));
    compareTestMsg(std::make_shared<gz::msgs::Vector3d>(_msg->position(i)));
    compareTestMsg(std::make_shared<gz::msgs::Vector3d>(_msg->normal(i)));
    compareTestMsg(std::make_shared<gz::msgs::JointWrench>(_msg->wrench(i)));
  }
}

void createTestMsg(gz::msgs::Contacts & _msg)
{
  gz::msgs::Header header_msg;
  gz::msgs::Contact contact_msg;

  createTestMsg(header_msg);
  createTestMsg(contact_msg);

  _msg.mutable_header()->CopyFrom(header_msg);
  _msg.clear_contact();
  for (int i = 0; i < 10; i++) {
    auto contact = _msg.add_contact();
    contact->mutable_collision1()->CopyFrom(contact_msg.collision1());
    contact->mutable_collision2()->CopyFrom(contact_msg.collision2());
    contact->mutable_position()->CopyFrom(contact_msg.position());
    contact->mutable_normal()->CopyFrom(contact_msg.normal());
    contact->mutable_wrench()->CopyFrom(contact_msg.wrench());
    contact->mutable_depth()->CopyFrom(contact_msg.depth());
  }
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Contacts> & _msg)
{
  gz::msgs::Contacts expected_msg;
  createTestMsg(expected_msg);
  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));
  EXPECT_EQ(expected_msg.contact_size(), _msg->contact_size());
  for (int i = 0; i < expected_msg.contact_size(); i++) {
    compareTestMsg(std::make_shared<gz::msgs::Contact>(_msg->contact(i)));
  }
}

<<<<<<< HEAD
#if HAVE_DATAFRAME
=======
>>>>>>> 63b651a (Garden EOL (#662))
void createTestMsg(gz::msgs::Dataframe & _msg)
{
  gz::msgs::Header header_msg;
  createTestMsg(header_msg);
  _msg.mutable_header()->CopyFrom(header_msg);

  auto * rssiPtr = _msg.mutable_header()->add_data();
  rssiPtr->set_key("rssi");
  rssiPtr->add_value("-10.3");

  _msg.set_src_address("localhost:8080");
  _msg.set_dst_address("localhost:8081");
  _msg.set_data(std::string(150, '1'));
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Dataframe> & _msg)
{
  gz::msgs::Dataframe expected_msg;
  createTestMsg(expected_msg);
  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));

  ASSERT_GT(_msg->header().data_size(), 0);
  bool rssiFound = false;
  for (auto i = 0; i < _msg->header().data_size(); ++i) {
    if (_msg->header().data(i).key() == "rssi" &&
      _msg->header().data(i).value_size() > 0)
    {
      EXPECT_EQ(0u, _msg->header().data(i).value(0).find("-10.3"));
      rssiFound = true;
    }
  }

  EXPECT_TRUE(rssiFound);
  EXPECT_EQ(expected_msg.src_address(), _msg->src_address());
  EXPECT_EQ(expected_msg.dst_address(), _msg->dst_address());
  EXPECT_EQ(expected_msg.data(), _msg->data());
}
<<<<<<< HEAD
#endif  // HAVE_DATAFRAME
=======
>>>>>>> 63b651a (Garden EOL (#662))

void createTestMsg(gz::msgs::Image & _msg)
{
  gz::msgs::Header header_msg;
  createTestMsg(header_msg);

  _msg.mutable_header()->CopyFrom(header_msg);
  _msg.set_width(320);
  _msg.set_height(240);
  _msg.set_pixel_format_type(gz::msgs::PixelFormatType::RGB_INT8);
  _msg.set_step(_msg.width() * 3);
  _msg.set_data(std::string(_msg.height() * _msg.step(), '1'));
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Image> & _msg)
{
  gz::msgs::Image expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));
  EXPECT_EQ(expected_msg.width(), _msg->width());
  EXPECT_EQ(expected_msg.height(), _msg->height());
  EXPECT_EQ(expected_msg.pixel_format_type(), _msg->pixel_format_type());
  EXPECT_EQ(expected_msg.step(), _msg->step());
  EXPECT_EQ(expected_msg.data(), _msg->data());
}

void createTestMsg(gz::msgs::CameraInfo & _msg)
{
  gz::msgs::Header header_msg;
  createTestMsg(header_msg);

  _msg.mutable_header()->CopyFrom(header_msg);
  _msg.set_width(320);
  _msg.set_height(240);

  auto distortion = _msg.mutable_distortion();
  distortion->set_model(gz::msgs::CameraInfo::Distortion::PLUMB_BOB);
  distortion->add_k(1);
  distortion->add_k(2);
  distortion->add_k(3);
  distortion->add_k(4);
  distortion->add_k(5);

  auto intrinsics = _msg.mutable_intrinsics();
  intrinsics->add_k(1);
  intrinsics->add_k(0);
  intrinsics->add_k(0);
  intrinsics->add_k(0);
  intrinsics->add_k(1);
  intrinsics->add_k(0);
  intrinsics->add_k(0);
  intrinsics->add_k(0);
  intrinsics->add_k(1);

  auto projection = _msg.mutable_projection();
  projection->add_p(1);
  projection->add_p(0);
  projection->add_p(0);
  projection->add_p(0);
  projection->add_p(0);
  projection->add_p(1);
  projection->add_p(0);
  projection->add_p(0);
  projection->add_p(0);
  projection->add_p(0);
  projection->add_p(1);
  projection->add_p(0);

  _msg.add_rectification_matrix(1);
  _msg.add_rectification_matrix(0);
  _msg.add_rectification_matrix(0);
  _msg.add_rectification_matrix(0);
  _msg.add_rectification_matrix(1);
  _msg.add_rectification_matrix(0);
  _msg.add_rectification_matrix(0);
  _msg.add_rectification_matrix(0);
  _msg.add_rectification_matrix(1);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::CameraInfo> & _msg)
{
  gz::msgs::CameraInfo expected_msg;
  createTestMsg(expected_msg);

  ASSERT_TRUE(expected_msg.has_header());
  ASSERT_TRUE(_msg->has_header());

  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));
  EXPECT_EQ(expected_msg.width(), _msg->width());
  EXPECT_EQ(expected_msg.height(), _msg->height());

  ASSERT_TRUE(expected_msg.has_distortion());
  ASSERT_TRUE(_msg->has_distortion());

  auto distortion = _msg->distortion();
  auto expected_distortion = expected_msg.distortion();
  EXPECT_EQ(expected_distortion.model(), distortion.model());
  ASSERT_EQ(expected_distortion.k_size(), distortion.k_size());
  for (auto i = 0; i < expected_distortion.k_size(); ++i) {
    EXPECT_EQ(expected_distortion.k(i), distortion.k(i));
  }

  ASSERT_TRUE(expected_msg.has_intrinsics());
  ASSERT_TRUE(_msg->has_intrinsics());

  auto intrinsics = _msg->intrinsics();
  auto expected_intrinsics = expected_msg.intrinsics();
  ASSERT_EQ(expected_intrinsics.k_size(), intrinsics.k_size());
  for (auto i = 0; i < expected_intrinsics.k_size(); ++i) {
    EXPECT_EQ(expected_intrinsics.k(i), intrinsics.k(i));
  }

  ASSERT_TRUE(expected_msg.has_projection());
  ASSERT_TRUE(_msg->has_projection());

  auto projection = _msg->projection();
  auto expected_projection = expected_msg.projection();
  ASSERT_EQ(expected_projection.p_size(), projection.p_size());
  for (auto i = 0; i < expected_projection.p_size(); ++i) {
    EXPECT_EQ(expected_projection.p(i), projection.p(i));
  }

  ASSERT_EQ(expected_msg.rectification_matrix_size(), _msg->rectification_matrix_size());
  for (auto i = 0; i < expected_msg.rectification_matrix_size(); ++i) {
    EXPECT_EQ(expected_msg.rectification_matrix(i), _msg->rectification_matrix(i));
  }
}

void createTestMsg(gz::msgs::FluidPressure & _msg)
{
  gz::msgs::Header header_msg;
  createTestMsg(header_msg);

  _msg.mutable_header()->CopyFrom(header_msg);
  _msg.set_pressure(0.123);
  _msg.set_variance(0.456);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::FluidPressure> & _msg)
{
  gz::msgs::FluidPressure expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));
  EXPECT_FLOAT_EQ(expected_msg.pressure(), _msg->pressure());
  EXPECT_FLOAT_EQ(expected_msg.variance(), _msg->variance());
}

void createTestMsg(gz::msgs::IMU & _msg)
{
  gz::msgs::Header header_msg;
  gz::msgs::Quaternion quaternion_msg;
  gz::msgs::Vector3d vector3_msg;

  createTestMsg(header_msg);
  createTestMsg(quaternion_msg);
  createTestMsg(vector3_msg);

  _msg.mutable_header()->CopyFrom(header_msg);
  _msg.mutable_orientation()->CopyFrom(quaternion_msg);
  _msg.mutable_angular_velocity()->CopyFrom(vector3_msg);
  _msg.mutable_linear_acceleration()->CopyFrom(vector3_msg);
#ifdef GZ_MSGS_IMU_HAS_COVARIANCE
  for (int i = 0; i < 9; i++) {
    _msg.mutable_orientation_covariance()->add_data(i + 1);
    _msg.mutable_angular_velocity_covariance()->add_data(i + 1);
    _msg.mutable_linear_acceleration_covariance()->add_data(i + 1);
  }
#endif
}

void compareTestMsg(const std::shared_ptr<gz::msgs::IMU> & _msg)
{
  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));
  compareTestMsg(std::make_shared<gz::msgs::Quaternion>(_msg->orientation()));
  compareTestMsg(std::make_shared<gz::msgs::Vector3d>(_msg->angular_velocity()));
  compareTestMsg(std::make_shared<gz::msgs::Vector3d>(_msg->linear_acceleration()));
#ifdef GZ_MSGS_IMU_HAS_COVARIANCE
  for (int i = 0; i < 9; i++) {
    EXPECT_EQ(_msg->orientation_covariance().data(i), i + 1);
    EXPECT_EQ(_msg->angular_velocity_covariance().data(i), i + 1);
    EXPECT_EQ(_msg->linear_acceleration_covariance().data(i), i + 1);
  }
#endif
}

void createTestMsg(gz::msgs::Axis & _msg)
{
  _msg.set_position(1.0);
  _msg.set_velocity(2.0);
  _msg.set_force(3.0);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Axis> & _msg)
{
  gz::msgs::Axis expected_msg;
  createTestMsg(expected_msg);

  EXPECT_DOUBLE_EQ(expected_msg.position(), _msg->position());
  EXPECT_DOUBLE_EQ(expected_msg.velocity(), _msg->velocity());
  EXPECT_DOUBLE_EQ(expected_msg.force(), _msg->force());
}

void createTestMsg(gz::msgs::Model & _msg)
{
  gz::msgs::Header header_msg;
  createTestMsg(header_msg);
  _msg.mutable_header()->CopyFrom(header_msg);

  for (auto i = 0; i < 3; ++i) {
    auto newJoint = _msg.add_joint();
    newJoint->set_name("joint_" + std::to_string(i));

    gz::msgs::Axis axis_msg;
    createTestMsg(axis_msg);
    newJoint->mutable_axis1()->CopyFrom(axis_msg);
  }
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Model> & _msg)
{
  gz::msgs::Model expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));

  ASSERT_EQ(expected_msg.joint_size(), _msg->joint_size());
  for (auto i = 0; i < _msg->joint_size(); ++i) {
    EXPECT_EQ(expected_msg.joint(i).name(), _msg->joint(i).name());
    compareTestMsg(std::make_shared<gz::msgs::Axis>(_msg->joint(i).axis1()));
  }
}

void createTestMsg(gz::msgs::Joy & _msg)
{
  gz::msgs::Header header_msg;

  createTestMsg(header_msg);
  _msg.mutable_header()->CopyFrom(header_msg);

  for (int i = 0u; i < 3; ++i) {
    _msg.add_axes(0.5);
    _msg.add_buttons(0);
  }
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Joy> & _msg)
{
  gz::msgs::Joy expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));

  for (int i = 0; i < expected_msg.axes_size(); ++i) {
    EXPECT_FLOAT_EQ(expected_msg.axes(i), _msg->axes(i));
  }

  for (int i = 0; i < expected_msg.buttons_size(); ++i) {
    EXPECT_EQ(expected_msg.buttons(i), _msg->buttons(i));
  }
}

void createTestMsg(gz::msgs::LaserScan & _msg)
{
  gz::msgs::Header header_msg;
  createTestMsg(header_msg);

  const unsigned int num_readings = 100u;
  _msg.mutable_header()->CopyFrom(header_msg);
  _msg.set_frame("frame_id_value");
  _msg.set_angle_min(-1.57);
  _msg.set_angle_max(1.57);
  _msg.set_angle_step(3.14 / num_readings);
  _msg.set_range_min(1);
  _msg.set_range_max(2);
  _msg.set_count(num_readings);
  _msg.set_vertical_angle_min(0);
  _msg.set_vertical_angle_max(0);
  _msg.set_vertical_angle_step(0);
  _msg.set_vertical_count(0);

  for (auto i = 0u; i < _msg.count(); ++i) {
    _msg.add_ranges(0);
    _msg.add_intensities(1);
  }
}

void compareTestMsg(const std::shared_ptr<gz::msgs::LaserScan> & _msg)
{
  gz::msgs::LaserScan expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));
  EXPECT_FLOAT_EQ(expected_msg.angle_min(), _msg->angle_min());
  EXPECT_FLOAT_EQ(expected_msg.angle_max(), _msg->angle_max());
  EXPECT_FLOAT_EQ(expected_msg.angle_step(), _msg->angle_step());
  EXPECT_DOUBLE_EQ(expected_msg.range_min(), _msg->range_min());
  EXPECT_DOUBLE_EQ(expected_msg.range_max(), _msg->range_max());
  EXPECT_EQ(expected_msg.count(), _msg->count());
  EXPECT_DOUBLE_EQ(
    expected_msg.vertical_angle_min(),
    _msg->vertical_angle_min());
  EXPECT_DOUBLE_EQ(
    expected_msg.vertical_angle_max(),
    _msg->vertical_angle_max());
  EXPECT_DOUBLE_EQ(
    expected_msg.vertical_angle_step(),
    _msg->vertical_angle_step());
  EXPECT_EQ(expected_msg.vertical_count(), _msg->vertical_count());
  EXPECT_EQ(expected_msg.ranges_size(), _msg->ranges_size());
  EXPECT_EQ(expected_msg.intensities_size(), _msg->intensities_size());

  for (auto i = 0u; i < _msg->count(); ++i) {
    EXPECT_DOUBLE_EQ(expected_msg.ranges(i), _msg->ranges(i));
    EXPECT_DOUBLE_EQ(expected_msg.intensities(i), _msg->intensities(i));
  }
}

void createTestMsg(gz::msgs::Magnetometer & _msg)
{
  gz::msgs::Header header_msg;
  gz::msgs::Vector3d vector3_msg;

  createTestMsg(header_msg);
  createTestMsg(vector3_msg);

  _msg.mutable_header()->CopyFrom(header_msg);
  _msg.mutable_field_tesla()->CopyFrom(vector3_msg);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Magnetometer> & _msg)
{
  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));
  compareTestMsg(std::make_shared<gz::msgs::Vector3d>(_msg->field_tesla()));
}

void createTestMsg(gz::msgs::NavSat & _msg)
{
  gz::msgs::Header header_msg;
  createTestMsg(header_msg);

  _msg.mutable_header()->CopyFrom(header_msg);
  _msg.set_frame_id("frame_id_value");
  _msg.set_latitude_deg(0.00);
  _msg.set_longitude_deg(0.00);
  _msg.set_altitude(0.00);
  _msg.set_velocity_east(0.00);
  _msg.set_velocity_north(0.00);
  _msg.set_velocity_up(0.00);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::NavSat> & _msg)
{
  gz::msgs::NavSat expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));
  EXPECT_FLOAT_EQ(expected_msg.latitude_deg(), _msg->latitude_deg());
  EXPECT_FLOAT_EQ(expected_msg.longitude_deg(), _msg->longitude_deg());
  EXPECT_FLOAT_EQ(expected_msg.altitude(), _msg->altitude());
  EXPECT_FLOAT_EQ(expected_msg.velocity_east(), _msg->velocity_east());
  EXPECT_FLOAT_EQ(expected_msg.velocity_north(), _msg->velocity_north());
  EXPECT_FLOAT_EQ(expected_msg.velocity_up(), _msg->velocity_up());
}

void createTestMsg(gz::msgs::Actuators & _msg)
{
  gz::msgs::Header header_msg;

  createTestMsg(header_msg);
  _msg.mutable_header()->CopyFrom(header_msg);

  for (int i = 0u; i < 3; ++i) {
    _msg.add_position(0.5);
    _msg.add_velocity(1.0);
    _msg.add_normalized(0.2);
  }
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Actuators> & _msg)
{
  gz::msgs::Actuators expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));

  for (int i = 0; i < expected_msg.position_size(); ++i) {
    EXPECT_FLOAT_EQ(expected_msg.position(i), _msg->position(i));
  }

  for (int i = 0; i < expected_msg.velocity_size(); ++i) {
    EXPECT_FLOAT_EQ(expected_msg.velocity(i), _msg->velocity(i));
  }

  for (int i = 0; i < expected_msg.normalized_size(); ++i) {
    EXPECT_FLOAT_EQ(expected_msg.normalized(i), _msg->normalized(i));
  }
}

void createTestMsg(gz::msgs::Odometry & _msg)
{
  gz::msgs::Header header_msg;
  gz::msgs::Pose pose_msg;
  gz::msgs::Twist twist_msg;

  createTestMsg(header_msg);
  createTestMsg(pose_msg);
  createTestMsg(twist_msg);

  _msg.mutable_header()->CopyFrom(header_msg);
  _msg.mutable_pose()->CopyFrom(pose_msg);
  _msg.mutable_twist()->CopyFrom(twist_msg);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Odometry> & _msg)
{
  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));
  compareTestMsg(std::make_shared<gz::msgs::Pose>(_msg->pose()));
  compareTestMsg(std::make_shared<gz::msgs::Twist>(_msg->twist()));
}

void createTestMsg(gz::msgs::OdometryWithCovariance & _msg)
{
  gz::msgs::Header header_msg;
  gz::msgs::PoseWithCovariance pose_cov_msg;
  gz::msgs::TwistWithCovariance twist_cov_msg;

  createTestMsg(header_msg);
  createTestMsg(pose_cov_msg);
  createTestMsg(twist_cov_msg);

  _msg.mutable_header()->CopyFrom(header_msg);
  _msg.mutable_pose_with_covariance()->CopyFrom(pose_cov_msg);
  _msg.mutable_twist_with_covariance()->CopyFrom(twist_cov_msg);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::OdometryWithCovariance> & _msg)
{
  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));
  compareTestMsg(
    std::make_shared<gz::msgs::PoseWithCovariance>(
      _msg->
      pose_with_covariance()));
  compareTestMsg(
    std::make_shared<gz::msgs::TwistWithCovariance>(
      _msg->
      twist_with_covariance()));
}

void createTestMsg(gz::msgs::PointCloudPacked & _msg)
{
  gz::msgs::Header header_msg;

  createTestMsg(header_msg);

  _msg.mutable_header()->CopyFrom(header_msg);
  gz::msgs::PointCloudPacked::Field * field = _msg.add_field();
  field->set_name("x");
  field->set_offset(0);
  field->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
  field->set_count(1);

  uint32_t height = 4;
  uint32_t width = 2;

  _msg.set_height(height);
  _msg.set_width(width);
  _msg.set_is_bigendian(false);
  _msg.set_point_step(4);
  _msg.set_row_step(4 * width);
  _msg.set_is_dense(true);

  std::string * msgBuffer = _msg.mutable_data();
  msgBuffer->resize(_msg.row_step() * _msg.height());
  char * msgBufferIndex = msgBuffer->data();

  for (uint32_t j = 0; j < height; ++j) {
    for (uint32_t i = 0; i < width; ++i) {
      *reinterpret_cast<float *>(msgBufferIndex + _msg.field(0).offset()) =
        j * width + i;
      msgBufferIndex += _msg.point_step();
    }
  }
}

void compareTestMsg(const std::shared_ptr<gz::msgs::PointCloudPacked> & _msg)
{
  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));

  uint32_t height = 4;
  uint32_t width = 2;

  EXPECT_EQ(height, _msg->height());
  EXPECT_EQ(width, _msg->width());
  EXPECT_FALSE(_msg->is_bigendian());
  EXPECT_EQ(4u, _msg->point_step());
  EXPECT_EQ(4U * width, _msg->row_step());
  EXPECT_TRUE(_msg->is_dense());

  char * msgBufferIndex = const_cast<char *>(_msg->data().data());

  for (uint32_t j = 0; j < height; ++j) {
    for (uint32_t i = 0; i < width; ++i) {
      float * value =
        reinterpret_cast<float *>(msgBufferIndex + _msg->field(0).offset());

      EXPECT_FLOAT_EQ(static_cast<float>(j * width + i), *value);
      msgBufferIndex += _msg->point_step();
    }
  }
}

void createTestMsg(gz::msgs::BatteryState & _msg)
{
  gz::msgs::Header header_msg;
  createTestMsg(header_msg);

  _msg.mutable_header()->CopyFrom(header_msg);
  _msg.set_voltage(123);
  _msg.set_current(456);

  _msg.set_charge(789);
  _msg.set_capacity(321);
  _msg.set_percentage(654);
  _msg.set_power_supply_status(gz::msgs::BatteryState::DISCHARGING);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::BatteryState> & _msg)
{
  gz::msgs::BatteryState expected_msg;
  createTestMsg(expected_msg);

  ASSERT_TRUE(expected_msg.has_header());
  ASSERT_TRUE(_msg->has_header());

  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));
  EXPECT_EQ(expected_msg.voltage(), _msg->voltage());
  EXPECT_EQ(expected_msg.current(), _msg->current());
  EXPECT_EQ(expected_msg.charge(), _msg->charge());
  EXPECT_EQ(expected_msg.capacity(), _msg->capacity());
  EXPECT_EQ(expected_msg.percentage(), _msg->percentage());
  EXPECT_EQ(expected_msg.power_supply_status(), _msg->power_supply_status());
}

void createTestMsg(gz::msgs::JointTrajectoryPoint & _msg)
{
  const auto number_of_joints = 7;

  for (auto i = 0; i < number_of_joints; ++i) {
    _msg.add_positions(1.1 * i);
    _msg.add_velocities(2.2 * i);
    _msg.add_accelerations(3.3 * i);
    _msg.add_effort(4.4 * i);
  }
  auto time_from_start = _msg.mutable_time_from_start();
  time_from_start->set_sec(12345);
  time_from_start->set_nsec(67890);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::JointTrajectoryPoint> & _msg)
{
  gz::msgs::JointTrajectoryPoint expected_msg;
  createTestMsg(expected_msg);

  for (int i = 0; i < _msg->positions_size(); ++i) {
    EXPECT_EQ(expected_msg.positions(i), _msg->positions(i));
  }

  for (int i = 0; i < _msg->velocities_size(); ++i) {
    EXPECT_EQ(expected_msg.velocities(i), _msg->velocities(i));
  }

  for (int i = 0; i < _msg->accelerations_size(); ++i) {
    EXPECT_EQ(expected_msg.accelerations(i), _msg->accelerations(i));
  }

  for (int i = 0; i < _msg->effort_size(); ++i) {
    EXPECT_EQ(expected_msg.effort(i), _msg->effort(i));
  }

  EXPECT_EQ(expected_msg.time_from_start().sec(), _msg->time_from_start().sec());
  EXPECT_EQ(expected_msg.time_from_start().nsec(), _msg->time_from_start().nsec());
}

void createTestMsg(gz::msgs::JointTrajectory & _msg)
{
  const auto number_of_joints = 7;
  const auto number_of_trajectory_points = 10;

  gz::msgs::Header header_msg;
  createTestMsg(header_msg);
  _msg.mutable_header()->CopyFrom(header_msg);

  for (auto i = 0; i < number_of_joints; ++i) {
    _msg.add_joint_names("joint_" + std::to_string(i));
  }

  for (auto j = 0; j < number_of_trajectory_points; ++j) {
    gz::msgs::JointTrajectoryPoint point;
    createTestMsg(point);
    _msg.add_points();
    _msg.mutable_points(j)->CopyFrom(point);
  }
}

void compareTestMsg(const std::shared_ptr<gz::msgs::JointTrajectory> & _msg)
{
  gz::msgs::JointTrajectory expected_msg;
  createTestMsg(expected_msg);

  ASSERT_TRUE(expected_msg.has_header());
  ASSERT_TRUE(_msg->has_header());

  for (int i = 0; i < _msg->joint_names_size(); ++i) {
    EXPECT_EQ(expected_msg.joint_names(i), _msg->joint_names(i));
  }

  for (int i = 0; i < _msg->points_size(); ++i) {
    compareTestMsg(std::make_shared<gz::msgs::JointTrajectoryPoint>(_msg->points(i)));
  }
}

void createTestMsg(gz::msgs::Light & _msg)
{
  gz::msgs::Header header_msg;
  gz::msgs::Pose pose_msg;
  gz::msgs::Color diffuse_msg;
  gz::msgs::Color specular_msg;
  gz::msgs::Vector3d direction_msg;

  createTestMsg(header_msg);
  createTestMsg(pose_msg);
  createTestMsg(diffuse_msg);
  createTestMsg(specular_msg);
  createTestMsg(direction_msg);

  _msg.mutable_header()->CopyFrom(header_msg);
  _msg.mutable_pose()->CopyFrom(pose_msg);
  _msg.mutable_diffuse()->CopyFrom(diffuse_msg);
  _msg.mutable_specular()->CopyFrom(specular_msg);
  _msg.mutable_direction()->CopyFrom(direction_msg);

  _msg.set_name("test_light");
  _msg.set_type(gz::msgs::Light_LightType::Light_LightType_SPOT);

  _msg.set_attenuation_constant(0.2);
  _msg.set_attenuation_linear(0.4);
  _msg.set_attenuation_quadratic(0.6);
  _msg.set_range(25.0);
  _msg.set_cast_shadows(true);
  _msg.set_spot_inner_angle(0.3);
  _msg.set_spot_outer_angle(0.6);
  _msg.set_spot_falloff(10.0);

  _msg.set_id(24);

  _msg.set_parent_id(6);

  _msg.set_intensity(125.0);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Light> & _msg)
{
  gz::msgs::Light expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));
  compareTestMsg(std::make_shared<gz::msgs::Pose>(_msg->pose()));
  compareTestMsg(std::make_shared<gz::msgs::Color>(_msg->diffuse()));
  compareTestMsg(std::make_shared<gz::msgs::Color>(_msg->specular()));
  compareTestMsg(std::make_shared<gz::msgs::Vector3d>(_msg->direction()));

  EXPECT_EQ(expected_msg.name(), _msg->name());
  EXPECT_EQ(expected_msg.type(), _msg->type());

  EXPECT_FLOAT_EQ(expected_msg.attenuation_constant(), _msg->attenuation_constant());
  EXPECT_FLOAT_EQ(expected_msg.attenuation_linear(), _msg->attenuation_linear());
  EXPECT_FLOAT_EQ(expected_msg.attenuation_quadratic(), _msg->attenuation_quadratic());
  EXPECT_FLOAT_EQ(expected_msg.range(), _msg->range());
  EXPECT_EQ(expected_msg.cast_shadows(), _msg->cast_shadows());
  EXPECT_FLOAT_EQ(expected_msg.spot_inner_angle(), _msg->spot_inner_angle());
  EXPECT_FLOAT_EQ(expected_msg.spot_outer_angle(), _msg->spot_outer_angle());
  EXPECT_FLOAT_EQ(expected_msg.spot_falloff(), _msg->spot_falloff());

  EXPECT_EQ(expected_msg.id(), _msg->id());

  EXPECT_EQ(expected_msg.parent_id(), _msg->parent_id());

  EXPECT_FLOAT_EQ(expected_msg.intensity(), _msg->intensity());
}

<<<<<<< HEAD
#if HAVE_MATERIALCOLOR
=======
>>>>>>> 63b651a (Garden EOL (#662))
void createTestMsg(gz::msgs::MaterialColor & _msg)
{
  createTestMsg(*_msg.mutable_header());
  createTestMsg(*_msg.mutable_entity());
  createTestMsg(*_msg.mutable_ambient());
  createTestMsg(*_msg.mutable_diffuse());
  createTestMsg(*_msg.mutable_specular());
  createTestMsg(*_msg.mutable_emissive());

  _msg.set_shininess(1.0);
  _msg.set_entity_match(gz::msgs::MaterialColor::EntityMatch::MaterialColor_EntityMatch_ALL);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::MaterialColor> & _msg)
{
  gz::msgs::MaterialColor expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));
  compareTestMsg(std::make_shared<gz::msgs::Entity>(_msg->entity()));
  compareTestMsg(std::make_shared<gz::msgs::Color>(_msg->ambient()));
  compareTestMsg(std::make_shared<gz::msgs::Color>(_msg->diffuse()));
  compareTestMsg(std::make_shared<gz::msgs::Color>(_msg->specular()));
  compareTestMsg(std::make_shared<gz::msgs::Color>(_msg->emissive()));

  EXPECT_EQ(expected_msg.shininess(), _msg->shininess());
  EXPECT_EQ(expected_msg.entity_match(), _msg->entity_match());
}
<<<<<<< HEAD
#endif  // HAVE_MATERIALCOLOR
=======
>>>>>>> 63b651a (Garden EOL (#662))

void createTestMsg(gz::msgs::GUICamera & _msg)
{
  gz::msgs::Header header_msg;
  gz::msgs::TrackVisual track_visual_msg;
  gz::msgs::Pose pose_msg;

  createTestMsg(header_msg);
  createTestMsg(track_visual_msg);
  createTestMsg(pose_msg);

  _msg.mutable_header()->CopyFrom(header_msg);
  _msg.mutable_track()->CopyFrom(track_visual_msg);
  _msg.mutable_pose()->CopyFrom(pose_msg);

  _msg.set_name("test_gui_camera");
  _msg.set_view_controller("test_gui_camera_view_controller");
  _msg.set_projection_type("test_gui_camera_projection_type");
}

/// \brief Compare a message with the populated for testing.
/// \param[in] _msg The message to compare.
void compareTestMsg(const std::shared_ptr<gz::msgs::GUICamera> & _msg)
{
  gz::msgs::GUICamera expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));
  compareTestMsg(std::make_shared<gz::msgs::Pose>(_msg->pose()));
  compareTestMsg(std::make_shared<gz::msgs::TrackVisual>(_msg->track()));

  EXPECT_EQ(expected_msg.name(), _msg->name());
  EXPECT_EQ(expected_msg.view_controller(), _msg->view_controller());
  EXPECT_EQ(expected_msg.projection_type(), _msg->projection_type());
}

void createTestMsg(gz::msgs::StringMsg_V & _msg)
{
  gz::msgs::Header header_msg;

  createTestMsg(header_msg);

  _msg.mutable_header()->CopyFrom(header_msg);

  auto * data = _msg.add_data();
  *data = "test_string_msg_v_data";
}

void compareTestMsg(const std::shared_ptr<gz::msgs::StringMsg_V> & _msg)
{
  gz::msgs::StringMsg_V expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));

  ASSERT_EQ(expected_msg.data_size(), _msg->data_size());
  EXPECT_EQ(expected_msg.data(0), _msg->data(0));
}

void createTestMsg(gz::msgs::Time & _msg)
{
  _msg.set_sec(12);
  _msg.set_nsec(150);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::Time> & _msg)
{
  gz::msgs::Time expected_msg;
  createTestMsg(expected_msg);

  EXPECT_EQ(expected_msg.sec(), _msg->sec());
  EXPECT_EQ(expected_msg.nsec(), _msg->nsec());
}

void createTestMsg(gz::msgs::TrackVisual & _msg)
{
  gz::msgs::Header header_msg;
  gz::msgs::Vector3d xyz_msg;

  createTestMsg(header_msg);
  createTestMsg(xyz_msg);

  _msg.mutable_header()->CopyFrom(header_msg);
  _msg.mutable_xyz()->CopyFrom(xyz_msg);

  _msg.set_name("test_track_visual");
  _msg.set_id(15);
  _msg.set_inherit_orientation(true);
  _msg.set_min_dist(1.1);
  _msg.set_max_dist(1.5);
  _msg.set_static_(true);
  _msg.set_use_model_frame(true);
  _msg.set_inherit_yaw(true);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::TrackVisual> & _msg)
{
  gz::msgs::TrackVisual expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));
  compareTestMsg(std::make_shared<gz::msgs::Vector3d>(_msg->xyz()));

  EXPECT_EQ(expected_msg.name(), _msg->name());
  EXPECT_EQ(expected_msg.id(), _msg->id());
  EXPECT_EQ(expected_msg.inherit_orientation(), _msg->inherit_orientation());
  EXPECT_EQ(expected_msg.min_dist(), _msg->min_dist());
  EXPECT_EQ(expected_msg.max_dist(), _msg->max_dist());
  EXPECT_EQ(expected_msg.static_(), _msg->static_());
  EXPECT_EQ(expected_msg.use_model_frame(), _msg->use_model_frame());
  EXPECT_EQ(expected_msg.inherit_yaw(), _msg->inherit_yaw());
}

void createTestMsg(gz::msgs::VideoRecord & _msg)
{
  gz::msgs::Header header_msg;

  createTestMsg(header_msg);

  _msg.mutable_header()->CopyFrom(header_msg);

  _msg.set_start(true);
  _msg.set_stop(true);
  _msg.set_format("test_video_record_format");
  _msg.set_save_filename("test_video_record_save_filename");
}

void compareTestMsg(const std::shared_ptr<gz::msgs::VideoRecord> & _msg)
{
  gz::msgs::VideoRecord expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));

  EXPECT_EQ(expected_msg.start(), _msg->start());
  EXPECT_EQ(expected_msg.stop(), _msg->stop());
  EXPECT_EQ(expected_msg.format(), _msg->format());
  EXPECT_EQ(expected_msg.save_filename(), _msg->save_filename());
}

void createTestMsg(gz::msgs::AnnotatedAxisAligned2DBox & _msg)
{
  gz::msgs::Header header_msg;

  createTestMsg(header_msg);

  _msg.mutable_header()->CopyFrom(header_msg);

  _msg.set_label(1);

  gz::msgs::AxisAligned2DBox * box = new gz::msgs::AxisAligned2DBox();
  gz::msgs::Vector2d * min_corner = new gz::msgs::Vector2d();
  gz::msgs::Vector2d * max_corner = new gz::msgs::Vector2d();

  min_corner->set_x(2.0);
  min_corner->set_y(2.0);
  max_corner->set_x(4.0);
  max_corner->set_y(6.0);
  box->set_allocated_min_corner(min_corner);
  box->set_allocated_max_corner(max_corner);
  _msg.set_allocated_box(box);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::AnnotatedAxisAligned2DBox> & _msg)
{
  gz::msgs::AnnotatedAxisAligned2DBox expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));

  EXPECT_EQ(expected_msg.label(), _msg->label());

  gz::msgs::AxisAligned2DBox expected_box = expected_msg.box();
  gz::msgs::Vector2d expected_min_corner = expected_box.min_corner();
  gz::msgs::Vector2d expected_max_corner = expected_box.max_corner();

  gz::msgs::AxisAligned2DBox box = _msg->box();
  gz::msgs::Vector2d min_corner = box.min_corner();
  gz::msgs::Vector2d max_corner = box.max_corner();

  EXPECT_EQ(expected_min_corner.x(), min_corner.x());
  EXPECT_EQ(expected_min_corner.y(), min_corner.y());
  EXPECT_EQ(expected_max_corner.x(), max_corner.x());
  EXPECT_EQ(expected_max_corner.y(), max_corner.y());
}

void createTestMsg(gz::msgs::AnnotatedAxisAligned2DBox_V & _msg)
{
  gz::msgs::Header header_msg;

  createTestMsg(header_msg);

  _msg.mutable_header()->CopyFrom(header_msg);

  for (size_t i = 0; i < 4; i++) {
    gz::msgs::AnnotatedAxisAligned2DBox * box = _msg.add_annotated_box();
    createTestMsg(*box);
  }
}

void compareTestMsg(const std::shared_ptr<gz::msgs::AnnotatedAxisAligned2DBox_V> & _msg)
{
  gz::msgs::AnnotatedAxisAligned2DBox_V expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));

  for (size_t i = 0; i < 4; i++) {
    compareTestMsg(
      std::make_shared<gz::msgs::AnnotatedAxisAligned2DBox>(
        _msg->annotated_box(
          i)));
  }
}

void createTestMsg(gz::msgs::AnnotatedOriented3DBox & _msg)
{
  gz::msgs::Header header_msg;

  createTestMsg(header_msg);

  _msg.mutable_header()->CopyFrom(header_msg);

  _msg.set_label(1);

  gz::msgs::Oriented3DBox * box = new gz::msgs::Oriented3DBox();
  gz::msgs::Vector3d * center = new gz::msgs::Vector3d();
  gz::msgs::Vector3d * size = new gz::msgs::Vector3d();
  gz::msgs::Quaternion * orientation = new gz::msgs::Quaternion();

  center->set_x(2.0);
  center->set_y(4.0);
  center->set_z(6.0);
  size->set_x(3.0);
  size->set_y(5.0);
  size->set_z(7.0);
  orientation->set_x(0.733);
  orientation->set_y(0.462);
  orientation->set_z(0.191);
  orientation->set_w(0.462);

  box->set_allocated_center(center);
  box->set_allocated_boxsize(size);
  box->set_allocated_orientation(orientation);
  _msg.set_allocated_box(box);
}

void compareTestMsg(const std::shared_ptr<gz::msgs::AnnotatedOriented3DBox> & _msg)
{
  gz::msgs::AnnotatedOriented3DBox expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));

  EXPECT_EQ(expected_msg.label(), _msg->label());

  gz::msgs::Oriented3DBox box = _msg->box();
  gz::msgs::Vector3d center = box.center();
  gz::msgs::Vector3d size = box.boxsize();
  gz::msgs::Quaternion orientation = box.orientation();

  EXPECT_EQ(expected_msg.box().center().x(), center.x());
  EXPECT_EQ(expected_msg.box().center().y(), center.y());
  EXPECT_EQ(expected_msg.box().center().z(), center.z());
  EXPECT_EQ(expected_msg.box().boxsize().x(), size.x());
  EXPECT_EQ(expected_msg.box().boxsize().y(), size.y());
  EXPECT_EQ(expected_msg.box().boxsize().z(), size.z());
  EXPECT_EQ(expected_msg.box().orientation().w(), orientation.w());
  EXPECT_EQ(expected_msg.box().orientation().x(), orientation.x());
  EXPECT_EQ(expected_msg.box().orientation().y(), orientation.y());
  EXPECT_EQ(expected_msg.box().orientation().z(), orientation.z());
}

void createTestMsg(gz::msgs::AnnotatedOriented3DBox_V & _msg)
{
  gz::msgs::Header header_msg;

  createTestMsg(header_msg);

  _msg.mutable_header()->CopyFrom(header_msg);

  for (size_t i = 0; i < 4; i++) {
    gz::msgs::AnnotatedOriented3DBox * box = _msg.add_annotated_box();
    createTestMsg(*box);
  }
}

void compareTestMsg(const std::shared_ptr<gz::msgs::AnnotatedOriented3DBox_V> & _msg)
{
  gz::msgs::AnnotatedOriented3DBox_V expected_msg;
  createTestMsg(expected_msg);

  compareTestMsg(std::make_shared<gz::msgs::Header>(_msg->header()));

  for (size_t i = 0; i < 4; i++) {
    compareTestMsg(
      std::make_shared<gz::msgs::AnnotatedOriented3DBox>(
        _msg->annotated_box(
          i)));
  }
}

}  // namespace testing
}  // namespace ros_gz_bridge
