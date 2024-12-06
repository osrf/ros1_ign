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

#include <limits>

#include "convert/utils.hpp"
#include "ros_gz_bridge/convert/sensor_msgs.hpp"

#include "gz/msgs/config.hh"

#if GZ_MSGS_MAJOR_VERSION >= 10
#define GZ_MSGS_IMU_HAS_COVARIANCE
#endif

namespace ros_gz_bridge
{

template<>
void
convert_ros_to_gz(
  const sensor_msgs::msg::FluidPressure & ros_msg,
  gz::msgs::FluidPressure & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  gz_msg.set_pressure(ros_msg.fluid_pressure);
  gz_msg.set_variance(ros_msg.variance);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::FluidPressure & gz_msg,
  sensor_msgs::msg::FluidPressure & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  ros_msg.fluid_pressure = gz_msg.pressure();
  ros_msg.variance = gz_msg.variance();
}

template<>
void
convert_ros_to_gz(
  const sensor_msgs::msg::Image & ros_msg,
  gz::msgs::Image & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));

  gz_msg.set_width(ros_msg.width);
  gz_msg.set_height(ros_msg.height);

  unsigned int num_channels;
  unsigned int octets_per_channel;

  if (ros_msg.encoding == "mono8") {
    gz_msg.set_pixel_format_type(gz::msgs::PixelFormatType::L_INT8);
    num_channels = 1;
    octets_per_channel = 1u;
  } else if (ros_msg.encoding == "mono16") {
    gz_msg.set_pixel_format_type(gz::msgs::PixelFormatType::L_INT16);
    num_channels = 1;
    octets_per_channel = 2u;
  } else if (ros_msg.encoding == "rgb8") {
    gz_msg.set_pixel_format_type(gz::msgs::PixelFormatType::RGB_INT8);
    num_channels = 3;
    octets_per_channel = 1u;
  } else if (ros_msg.encoding == "rgba8") {
    gz_msg.set_pixel_format_type(gz::msgs::PixelFormatType::RGBA_INT8);
    num_channels = 4;
    octets_per_channel = 1u;
  } else if (ros_msg.encoding == "bgra8") {
    gz_msg.set_pixel_format_type(gz::msgs::PixelFormatType::BGRA_INT8);
    num_channels = 4;
    octets_per_channel = 1u;
  } else if (ros_msg.encoding == "rgb16") {
    gz_msg.set_pixel_format_type(gz::msgs::PixelFormatType::RGB_INT16);
    num_channels = 3;
    octets_per_channel = 2u;
  } else if (ros_msg.encoding == "bgr8") {
    gz_msg.set_pixel_format_type(gz::msgs::PixelFormatType::BGR_INT8);
    num_channels = 3;
    octets_per_channel = 1u;
  } else if (ros_msg.encoding == "bgr16") {
    gz_msg.set_pixel_format_type(gz::msgs::PixelFormatType::BGR_INT16);
    num_channels = 3;
    octets_per_channel = 2u;
  } else if (ros_msg.encoding == "32FC1") {
    gz_msg.set_pixel_format_type(gz::msgs::PixelFormatType::R_FLOAT32);
    num_channels = 1;
    octets_per_channel = 4u;
  } else {
    gz_msg.set_pixel_format_type(gz::msgs::PixelFormatType::UNKNOWN_PIXEL_FORMAT);
    std::cerr << "Unsupported pixel format [" << ros_msg.encoding << "]" << std::endl;
    return;
  }

  gz_msg.set_step(gz_msg.width() * num_channels * octets_per_channel);

  gz_msg.set_data(&(ros_msg.data[0]), gz_msg.step() * gz_msg.height());
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Image & gz_msg,
  sensor_msgs::msg::Image & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  ros_msg.height = gz_msg.height();
  ros_msg.width = gz_msg.width();

  unsigned int num_channels;
  unsigned int octets_per_channel;

  if (gz_msg.pixel_format_type() == gz::msgs::PixelFormatType::L_INT8) {
    ros_msg.encoding = "mono8";
    num_channels = 1;
    octets_per_channel = 1u;
  } else if (gz_msg.pixel_format_type() == gz::msgs::PixelFormatType::L_INT16) {
    ros_msg.encoding = "mono16";
    num_channels = 1;
    octets_per_channel = 2u;
  } else if (gz_msg.pixel_format_type() == gz::msgs::PixelFormatType::RGB_INT8) {
    ros_msg.encoding = "rgb8";
    num_channels = 3;
    octets_per_channel = 1u;
  } else if (gz_msg.pixel_format_type() == gz::msgs::PixelFormatType::RGBA_INT8) {
    ros_msg.encoding = "rgba8";
    num_channels = 4;
    octets_per_channel = 1u;
  } else if (gz_msg.pixel_format_type() == gz::msgs::PixelFormatType::BGRA_INT8) {
    ros_msg.encoding = "bgra8";
    num_channels = 4;
    octets_per_channel = 1u;
  } else if (gz_msg.pixel_format_type() == gz::msgs::PixelFormatType::RGB_INT16) {
    ros_msg.encoding = "rgb16";
    num_channels = 3;
    octets_per_channel = 2u;
  } else if (gz_msg.pixel_format_type() == gz::msgs::PixelFormatType::BGR_INT8) {
    ros_msg.encoding = "bgr8";
    num_channels = 3;
    octets_per_channel = 1u;
  } else if (gz_msg.pixel_format_type() == gz::msgs::PixelFormatType::BGR_INT16) {
    ros_msg.encoding = "bgr16";
    num_channels = 3;
    octets_per_channel = 2u;
  } else if (gz_msg.pixel_format_type() == gz::msgs::PixelFormatType::R_FLOAT32) {
    ros_msg.encoding = "32FC1";
    num_channels = 1;
    octets_per_channel = 4u;
  } else {
    std::cerr << "Unsupported pixel format [" << gz_msg.pixel_format_type() << "]" << std::endl;
    return;
  }

  ros_msg.is_bigendian = false;
  ros_msg.step = ros_msg.width * num_channels * octets_per_channel;
<<<<<<< HEAD

  auto count = ros_msg.step * ros_msg.height;
  ros_msg.data.resize(ros_msg.step * ros_msg.height);
  std::copy(
    gz_msg.data().begin(),
    gz_msg.data().begin() + count,
    ros_msg.data.begin());
=======
  ros_msg.data.resize(ros_msg.step * ros_msg.height);

  // Prefer memcpy over std::copy for performance reasons,
  // see https://github.com/gazebosim/ros_gz/pull/565
  memcpy(ros_msg.data.data(), gz_msg.data().c_str(), gz_msg.data().size());
>>>>>>> 63b651a (Garden EOL (#662))
}

template<>
void
convert_ros_to_gz(
  const sensor_msgs::msg::CameraInfo & ros_msg,
  gz::msgs::CameraInfo & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));

  gz_msg.set_width(ros_msg.width);
  gz_msg.set_height(ros_msg.height);

  auto distortion = gz_msg.mutable_distortion();
  if (ros_msg.distortion_model == "plumb_bob") {
    distortion->set_model(gz::msgs::CameraInfo::Distortion::PLUMB_BOB);
  } else if (ros_msg.distortion_model == "rational_polynomial") {
    distortion->set_model(gz::msgs::CameraInfo::Distortion::RATIONAL_POLYNOMIAL);
  } else if (ros_msg.distortion_model == "equidistant") {
    distortion->set_model(gz::msgs::CameraInfo::Distortion::EQUIDISTANT);
  } else {
    std::cerr << "Unsupported distortion model [" << ros_msg.distortion_model << "]" << std::endl;
  }

  for (double i : ros_msg.d) {
    distortion->add_k(i);
  }

  auto intrinsics = gz_msg.mutable_intrinsics();
  for (double i : ros_msg.k) {
    intrinsics->add_k(i);
  }

  auto projection = gz_msg.mutable_projection();
  for (double i : ros_msg.p) {
    projection->add_p(i);
  }

  for (auto i = 0u; i < ros_msg.r.size(); ++i) {
    gz_msg.add_rectification_matrix(ros_msg.r[i]);
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::CameraInfo & gz_msg,
  sensor_msgs::msg::CameraInfo & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  ros_msg.height = gz_msg.height();
  ros_msg.width = gz_msg.width();

  if (gz_msg.has_distortion()) {
    const auto & distortion = gz_msg.distortion();
    if (distortion.model() == gz::msgs::CameraInfo::Distortion::PLUMB_BOB) {
      ros_msg.distortion_model = "plumb_bob";
    } else if (distortion.model() == gz::msgs::CameraInfo::Distortion::RATIONAL_POLYNOMIAL) {
      ros_msg.distortion_model = "rational_polynomial";
    } else if (distortion.model() == gz::msgs::CameraInfo::Distortion::EQUIDISTANT) {
      ros_msg.distortion_model = "equidistant";
    } else {
      std::cerr << "Unsupported distortion model [" << distortion.model() << "]" << std::endl;
    }

    ros_msg.d.resize(distortion.k_size());
    for (auto i = 0; i < distortion.k_size(); ++i) {
      ros_msg.d[i] = distortion.k(i);
    }
  }

  if (gz_msg.has_intrinsics()) {
    const auto & intrinsics = gz_msg.intrinsics();

    for (auto i = 0; i < intrinsics.k_size(); ++i) {
      ros_msg.k[i] = intrinsics.k(i);
    }
  }

  if (gz_msg.has_projection()) {
    const auto & projection = gz_msg.projection();

    for (auto i = 0; i < projection.p_size(); ++i) {
      ros_msg.p[i] = projection.p(i);
    }
  }

  for (auto i = 0; i < gz_msg.rectification_matrix_size(); ++i) {
    ros_msg.r[i] = gz_msg.rectification_matrix(i);
  }
}

template<>
void
convert_ros_to_gz(
  const sensor_msgs::msg::Imu & ros_msg,
  gz::msgs::IMU & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));

  // ToDo: Verify that this is the expected value (probably not).
  gz_msg.set_entity_name(ros_msg.header.frame_id);

  convert_ros_to_gz(ros_msg.orientation, (*gz_msg.mutable_orientation()));
  convert_ros_to_gz(ros_msg.angular_velocity, (*gz_msg.mutable_angular_velocity()));
  convert_ros_to_gz(ros_msg.linear_acceleration, (*gz_msg.mutable_linear_acceleration()));

#ifdef GZ_MSGS_IMU_HAS_COVARIANCE
  for (const auto & elem : ros_msg.linear_acceleration_covariance) {
    gz_msg.mutable_linear_acceleration_covariance()->add_data(elem);
  }
  for (const auto & elem : ros_msg.orientation_covariance) {
    gz_msg.mutable_orientation_covariance()->add_data(elem);
  }
  for (const auto & elem : ros_msg.angular_velocity_covariance) {
    gz_msg.mutable_angular_velocity_covariance()->add_data(elem);
  }
#endif
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::IMU & gz_msg,
  sensor_msgs::msg::Imu & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  convert_gz_to_ros(gz_msg.orientation(), ros_msg.orientation);
  convert_gz_to_ros(gz_msg.angular_velocity(), ros_msg.angular_velocity);
  convert_gz_to_ros(gz_msg.linear_acceleration(), ros_msg.linear_acceleration);

#ifdef GZ_MSGS_IMU_HAS_COVARIANCE
  int data_size = gz_msg.linear_acceleration_covariance().data_size();
  if (data_size == 9) {
    for (int i = 0; i < data_size; ++i) {
      auto data = gz_msg.linear_acceleration_covariance().data()[i];
      ros_msg.linear_acceleration_covariance[i] = data;
    }
  }
  data_size = gz_msg.angular_velocity_covariance().data_size();
  if (data_size == 9) {
    for (int i = 0; i < data_size; ++i) {
      auto data = gz_msg.angular_velocity_covariance().data()[i];
      ros_msg.angular_velocity_covariance[i] = data;
    }
  }
  data_size = gz_msg.orientation_covariance().data_size();
  if (data_size == 9) {
    for (int i = 0; i < data_size; ++i) {
      auto data = gz_msg.orientation_covariance().data()[i];
      ros_msg.orientation_covariance[i] = data;
    }
  }
#endif
}

template<>
void
convert_ros_to_gz(
  const sensor_msgs::msg::JointState & ros_msg,
  gz::msgs::Model & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));

  for (auto i = 0u; i < ros_msg.position.size(); ++i) {
    auto newJoint = gz_msg.add_joint();
    newJoint->set_name(ros_msg.name[i]);
    newJoint->mutable_axis1()->set_position(ros_msg.position[i]);
    newJoint->mutable_axis1()->set_velocity(ros_msg.velocity[i]);
    newJoint->mutable_axis1()->set_force(ros_msg.effort[i]);
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Model & gz_msg,
  sensor_msgs::msg::JointState & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  for (auto i = 0; i < gz_msg.joint_size(); ++i) {
    ros_msg.name.push_back(gz_msg.joint(i).name());
    ros_msg.position.push_back(gz_msg.joint(i).axis1().position());
    ros_msg.velocity.push_back(gz_msg.joint(i).axis1().velocity());
    ros_msg.effort.push_back(gz_msg.joint(i).axis1().force());
  }
}

template<>
void
convert_ros_to_gz(
  const sensor_msgs::msg::Joy & ros_msg,
  gz::msgs::Joy & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));

  for (auto i = 0u; i < ros_msg.axes.size(); ++i) {
    gz_msg.add_axes(ros_msg.axes[i]);
  }

  for (auto i = 0u; i < ros_msg.buttons.size(); ++i) {
    gz_msg.add_buttons(ros_msg.buttons[i]);
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Joy & gz_msg,
  sensor_msgs::msg::Joy & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  for (auto i = 0; i < gz_msg.axes_size(); ++i) {
    ros_msg.axes.push_back(gz_msg.axes(i));
  }

  for (auto i = 0; i < gz_msg.buttons_size(); ++i) {
    ros_msg.buttons.push_back(gz_msg.buttons(i));
  }
}

template<>
void
convert_ros_to_gz(
  const sensor_msgs::msg::LaserScan & ros_msg,
  gz::msgs::LaserScan & gz_msg)
{
  const unsigned int num_readings =
    (ros_msg.angle_max - ros_msg.angle_min) / ros_msg.angle_increment;

  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  gz_msg.set_frame(ros_msg.header.frame_id);
  gz_msg.set_angle_min(ros_msg.angle_min);
  gz_msg.set_angle_max(ros_msg.angle_max);
  gz_msg.set_angle_step(ros_msg.angle_increment);
  gz_msg.set_range_min(ros_msg.range_min);
  gz_msg.set_range_max(ros_msg.range_max);
  gz_msg.set_count(num_readings);

  // Not supported in sensor_msgs::msg::LaserScan.
  gz_msg.set_vertical_angle_min(0.0);
  gz_msg.set_vertical_angle_max(0.0);
  gz_msg.set_vertical_angle_step(0.0);
  gz_msg.set_vertical_count(0u);

  for (auto i = 0u; i < gz_msg.count(); ++i) {
    gz_msg.add_ranges(ros_msg.ranges[i]);
    gz_msg.add_intensities(ros_msg.intensities[i]);
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::LaserScan & gz_msg,
  sensor_msgs::msg::LaserScan & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  ros_msg.header.frame_id = frame_id_gz_to_ros(gz_msg.frame());

  ros_msg.angle_min = gz_msg.angle_min();
  ros_msg.angle_max = gz_msg.angle_max();
  ros_msg.angle_increment = gz_msg.angle_step();

  // Not supported in gz::msgs::LaserScan.
  ros_msg.time_increment = 0.0;
  ros_msg.scan_time = 0.0;

  ros_msg.range_min = gz_msg.range_min();
  ros_msg.range_max = gz_msg.range_max();

  auto count = gz_msg.count();
  auto vertical_count = gz_msg.vertical_count();

  // If there are multiple vertical beams, use the one in the middle.
  size_t start = (vertical_count / 2) * count;

  // Copy ranges into ROS message.
  ros_msg.ranges.resize(count);
  std::copy(
    gz_msg.ranges().begin() + start,
    gz_msg.ranges().begin() + start + count,
    ros_msg.ranges.begin());

  // Copy intensities into ROS message.
  ros_msg.intensities.resize(count);
  std::copy(
    gz_msg.intensities().begin() + start,
    gz_msg.intensities().begin() + start + count,
    ros_msg.intensities.begin());
}

template<>
void
convert_ros_to_gz(
  const sensor_msgs::msg::MagneticField & ros_msg,
  gz::msgs::Magnetometer & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  convert_ros_to_gz(ros_msg.magnetic_field, (*gz_msg.mutable_field_tesla()));
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Magnetometer & gz_msg,
  sensor_msgs::msg::MagneticField & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  convert_gz_to_ros(gz_msg.field_tesla(), ros_msg.magnetic_field);

  // magnetic_field_covariance is not supported in gz::msgs::Magnetometer.
}

template<>
void
convert_ros_to_gz(
  const sensor_msgs::msg::NavSatFix & ros_msg,
  gz::msgs::NavSat & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  gz_msg.set_latitude_deg(ros_msg.latitude);
  gz_msg.set_longitude_deg(ros_msg.longitude);
  gz_msg.set_altitude(ros_msg.altitude);
  gz_msg.set_frame_id(ros_msg.header.frame_id);

  // Not supported in sensor_msgs::NavSatFix.
  gz_msg.set_velocity_east(0.0);
  gz_msg.set_velocity_north(0.0);
  gz_msg.set_velocity_up(0.0);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::NavSat & gz_msg,
  sensor_msgs::msg::NavSatFix & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  ros_msg.header.frame_id = frame_id_gz_to_ros(gz_msg.frame_id());
  ros_msg.latitude = gz_msg.latitude_deg();
  ros_msg.longitude = gz_msg.longitude_deg();
  ros_msg.altitude = gz_msg.altitude();

<<<<<<< HEAD
  // position_covariance is not supported in Ignition::Msgs::NavSat.
=======
  // position_covariance is not supported in gz::msgs::NavSat.
>>>>>>> 63b651a (Garden EOL (#662))
  ros_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  ros_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
}

template<>
void
convert_ros_to_gz(
  const sensor_msgs::msg::PointCloud2 & ros_msg,
  gz::msgs::PointCloudPacked & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));

  gz_msg.set_height(ros_msg.height);
  gz_msg.set_width(ros_msg.width);
  gz_msg.set_is_bigendian(ros_msg.is_bigendian);
  gz_msg.set_point_step(ros_msg.point_step);
  gz_msg.set_row_step(ros_msg.row_step);
  gz_msg.set_is_dense(ros_msg.is_dense);
  gz_msg.mutable_data()->resize(ros_msg.data.size());
  memcpy(gz_msg.mutable_data()->data(), ros_msg.data.data(), ros_msg.data.size());

  for (const auto & field : ros_msg.fields) {
    gz::msgs::PointCloudPacked::Field * pf = gz_msg.add_field();
    pf->set_name(field.name);
    pf->set_count(field.count);
    pf->set_offset(field.offset);
    switch (field.datatype) {
      default:
      case sensor_msgs::msg::PointField::INT8:
        pf->set_datatype(gz::msgs::PointCloudPacked::Field::INT8);
        break;
      case sensor_msgs::msg::PointField::UINT8:
        pf->set_datatype(gz::msgs::PointCloudPacked::Field::UINT8);
        break;
      case sensor_msgs::msg::PointField::INT16:
        pf->set_datatype(gz::msgs::PointCloudPacked::Field::INT16);
        break;
      case sensor_msgs::msg::PointField::UINT16:
        pf->set_datatype(gz::msgs::PointCloudPacked::Field::UINT16);
        break;
      case sensor_msgs::msg::PointField::INT32:
        pf->set_datatype(gz::msgs::PointCloudPacked::Field::INT32);
        break;
      case sensor_msgs::msg::PointField::UINT32:
        pf->set_datatype(gz::msgs::PointCloudPacked::Field::UINT32);
        break;
      case sensor_msgs::msg::PointField::FLOAT32:
        pf->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT32);
        break;
      case sensor_msgs::msg::PointField::FLOAT64:
        pf->set_datatype(gz::msgs::PointCloudPacked::Field::FLOAT64);
        break;
    }
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::PointCloudPacked & gz_msg,
  sensor_msgs::msg::PointCloud2 & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  ros_msg.height = gz_msg.height();
  ros_msg.width = gz_msg.width();
  ros_msg.is_bigendian = gz_msg.is_bigendian();
  ros_msg.point_step = gz_msg.point_step();
  ros_msg.row_step = gz_msg.row_step();
  ros_msg.is_dense = gz_msg.is_dense();
  ros_msg.data.resize(gz_msg.data().size());
  memcpy(ros_msg.data.data(), gz_msg.data().c_str(), gz_msg.data().size());

  for (int i = 0; i < gz_msg.field_size(); ++i) {
    sensor_msgs::msg::PointField pf;
    pf.name = gz_msg.field(i).name();
    pf.count = gz_msg.field(i).count();
    pf.offset = gz_msg.field(i).offset();
    switch (gz_msg.field(i).datatype()) {
      default:
      case gz::msgs::PointCloudPacked::Field::INT8:
        pf.datatype = sensor_msgs::msg::PointField::INT8;
        break;
      case gz::msgs::PointCloudPacked::Field::UINT8:
        pf.datatype = sensor_msgs::msg::PointField::UINT8;
        break;
      case gz::msgs::PointCloudPacked::Field::INT16:
        pf.datatype = sensor_msgs::msg::PointField::INT16;
        break;
      case gz::msgs::PointCloudPacked::Field::UINT16:
        pf.datatype = sensor_msgs::msg::PointField::UINT16;
        break;
      case gz::msgs::PointCloudPacked::Field::INT32:
        pf.datatype = sensor_msgs::msg::PointField::INT32;
        break;
      case gz::msgs::PointCloudPacked::Field::UINT32:
        pf.datatype = sensor_msgs::msg::PointField::UINT32;
        break;
      case gz::msgs::PointCloudPacked::Field::FLOAT32:
        pf.datatype = sensor_msgs::msg::PointField::FLOAT32;
        break;
      case gz::msgs::PointCloudPacked::Field::FLOAT64:
        pf.datatype = sensor_msgs::msg::PointField::FLOAT64;
        break;
    }
    ros_msg.fields.push_back(pf);
  }
}

template<>
void
convert_ros_to_gz(
  const sensor_msgs::msg::BatteryState & ros_msg,
  gz::msgs::BatteryState & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));

  gz_msg.set_voltage(ros_msg.voltage);
  gz_msg.set_current(ros_msg.current);
  gz_msg.set_charge(ros_msg.charge);
  gz_msg.set_capacity(ros_msg.capacity);
  gz_msg.set_percentage(ros_msg.percentage);

  switch (ros_msg.power_supply_status) {
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN:
      gz_msg.set_power_supply_status(gz::msgs::BatteryState::UNKNOWN);
      break;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING:
      gz_msg.set_power_supply_status(gz::msgs::BatteryState::CHARGING);
      break;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING:
      gz_msg.set_power_supply_status(gz::msgs::BatteryState::DISCHARGING);
      break;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING:
      gz_msg.set_power_supply_status(gz::msgs::BatteryState::NOT_CHARGING);
      break;
    case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL:
      gz_msg.set_power_supply_status(gz::msgs::BatteryState::FULL);
      break;
    default:
      std::cerr << "Unsupported power supply status [" << ros_msg.power_supply_status << "]\n";
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::BatteryState & gz_msg,
  sensor_msgs::msg::BatteryState & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  ros_msg.voltage = gz_msg.voltage();
  ros_msg.current = gz_msg.current();
  ros_msg.charge = gz_msg.charge();
  ros_msg.capacity = gz_msg.capacity();
  ros_msg.design_capacity = std::numeric_limits<double>::quiet_NaN();
  ros_msg.percentage = gz_msg.percentage();

  if (gz_msg.power_supply_status() == gz::msgs::BatteryState::UNKNOWN) {
    ros_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  } else if (gz_msg.power_supply_status() == gz::msgs::BatteryState::CHARGING) {
    ros_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  } else if (gz_msg.power_supply_status() == gz::msgs::BatteryState::DISCHARGING) {
    ros_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  } else if (gz_msg.power_supply_status() == gz::msgs::BatteryState::NOT_CHARGING) {
    ros_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
  } else if (gz_msg.power_supply_status() == gz::msgs::BatteryState::FULL) {
    ros_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
  } else {
    std::cerr << "Unsupported power supply status [" <<
      gz_msg.power_supply_status() << "]" << std::endl;
  }

  ros_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  ros_msg.power_supply_technology =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  ros_msg.present = true;
}

}  // namespace ros_gz_bridge
