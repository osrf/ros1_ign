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

#include "ros_gz_bridge/convert/ros_gz_interfaces.hpp"

namespace ros_gz_bridge
{

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::JointWrench & ros_msg,
  gz::msgs::JointWrench & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  gz_msg.set_body_1_name(ros_msg.body_1_name.data);
  gz_msg.set_body_2_name(ros_msg.body_2_name.data);
  gz_msg.set_body_1_id(ros_msg.body_1_id.data);
  gz_msg.set_body_2_id(ros_msg.body_2_id.data);
  convert_ros_to_gz(ros_msg.body_1_wrench, (*gz_msg.mutable_body_1_wrench()));
  convert_ros_to_gz(ros_msg.body_2_wrench, (*gz_msg.mutable_body_2_wrench()));
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::JointWrench & gz_msg,
  ros_gz_interfaces::msg::JointWrench & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  ros_msg.body_1_name.data = gz_msg.body_1_name();
  ros_msg.body_2_name.data = gz_msg.body_2_name();
  ros_msg.body_1_id.data = gz_msg.body_1_id();
  ros_msg.body_2_id.data = gz_msg.body_2_id();
  convert_gz_to_ros(gz_msg.body_1_wrench(), ros_msg.body_1_wrench);
  convert_gz_to_ros(gz_msg.body_2_wrench(), ros_msg.body_2_wrench);
}

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Altimeter & ros_msg,
  gz::msgs::Altimeter & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  gz_msg.set_vertical_position(ros_msg.vertical_position);
  gz_msg.set_vertical_velocity(ros_msg.vertical_velocity);
  gz_msg.set_vertical_reference(ros_msg.vertical_reference);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Altimeter & gz_msg,
  ros_gz_interfaces::msg::Altimeter & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  ros_msg.vertical_position = gz_msg.vertical_position();
  ros_msg.vertical_velocity = gz_msg.vertical_velocity();
  ros_msg.vertical_reference = gz_msg.vertical_reference();
}

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Entity & ros_msg,
  gz::msgs::Entity & gz_msg)
{
  gz_msg.set_id(ros_msg.id);
  gz_msg.set_name(ros_msg.name);
  switch (ros_msg.type) {
    case ros_gz_interfaces::msg::Entity::NONE:
      gz_msg.set_type(gz::msgs::Entity::NONE);
      break;
    case ros_gz_interfaces::msg::Entity::LIGHT:
      gz_msg.set_type(gz::msgs::Entity::LIGHT);
      break;
    case ros_gz_interfaces::msg::Entity::MODEL:
      gz_msg.set_type(gz::msgs::Entity::MODEL);
      break;
    case ros_gz_interfaces::msg::Entity::LINK:
      gz_msg.set_type(gz::msgs::Entity::LINK);
      break;
    case ros_gz_interfaces::msg::Entity::VISUAL:
      gz_msg.set_type(gz::msgs::Entity::VISUAL);
      break;
    case ros_gz_interfaces::msg::Entity::COLLISION:
      gz_msg.set_type(gz::msgs::Entity::COLLISION);
      break;
    case ros_gz_interfaces::msg::Entity::SENSOR:
      gz_msg.set_type(gz::msgs::Entity::SENSOR);
      break;
    case ros_gz_interfaces::msg::Entity::JOINT:
      gz_msg.set_type(gz::msgs::Entity::JOINT);
      break;
    default:
      std::cerr << "Unsupported entity type [" << ros_msg.type << "]\n";
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Entity & gz_msg,
  ros_gz_interfaces::msg::Entity & ros_msg)
{
  ros_msg.id = gz_msg.id();
  ros_msg.name = gz_msg.name();
  if (gz_msg.type() == gz::msgs::Entity::Type::Entity_Type_NONE) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::NONE;
  } else if (gz_msg.type() == gz::msgs::Entity::LIGHT) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::LIGHT;
  } else if (gz_msg.type() == gz::msgs::Entity::MODEL) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::MODEL;
  } else if (gz_msg.type() == gz::msgs::Entity::LINK) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::LINK;
  } else if (gz_msg.type() == gz::msgs::Entity::VISUAL) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::VISUAL;
  } else if (gz_msg.type() == gz::msgs::Entity::COLLISION) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::COLLISION;
  } else if (gz_msg.type() == gz::msgs::Entity::SENSOR) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::SENSOR;
  } else if (gz_msg.type() == gz::msgs::Entity::JOINT) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::JOINT;
  } else {
    std::cerr << "Unsupported Entity [" <<
      gz_msg.type() << "]" << std::endl;
  }
}

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::EntityWrench & ros_msg,
  gz::msgs::EntityWrench & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  convert_ros_to_gz(ros_msg.entity, (*gz_msg.mutable_entity()));
  convert_ros_to_gz(ros_msg.wrench, (*gz_msg.mutable_wrench()));
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::EntityWrench & gz_msg,
  ros_gz_interfaces::msg::EntityWrench & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  convert_gz_to_ros(gz_msg.entity(), ros_msg.entity);
  convert_gz_to_ros(gz_msg.wrench(), ros_msg.wrench);
}

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Contact & ros_msg,
  gz::msgs::Contact & gz_msg)
{
  convert_ros_to_gz(ros_msg.collision1, (*gz_msg.mutable_collision1()));
  convert_ros_to_gz(ros_msg.collision1, (*gz_msg.mutable_collision2()));
  gz_msg.clear_position();
  for (auto const & ros_position : ros_msg.positions) {
    auto gz_position = gz_msg.add_position();
    convert_ros_to_gz(ros_position, *gz_position);
  }
  gz_msg.clear_normal();
  for (const auto & ros_normal : ros_msg.normals) {
    auto gz_normal = gz_msg.add_normal();
    convert_ros_to_gz(ros_normal, *gz_normal);
  }
  for (const auto & ros_depth : ros_msg.depths) {
    gz_msg.add_depth(ros_depth);
  }
  gz_msg.clear_wrench();
  for (const auto & ros_wrench : ros_msg.wrenches) {
    auto gz_wrench = gz_msg.add_wrench();
    convert_ros_to_gz(ros_wrench, *gz_wrench);
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Contact & gz_msg,
  ros_gz_interfaces::msg::Contact & ros_msg)
{
  convert_gz_to_ros(gz_msg.collision1(), ros_msg.collision1);
  convert_gz_to_ros(gz_msg.collision2(), ros_msg.collision2);
  for (auto i = 0; i < gz_msg.position_size(); ++i) {
    geometry_msgs::msg::Vector3 ros_position;
    convert_gz_to_ros(gz_msg.position(i), ros_position);
    ros_msg.positions.push_back(ros_position);
  }
  for (auto i = 0; i < gz_msg.normal_size(); ++i) {
    geometry_msgs::msg::Vector3 ros_normal;
    convert_gz_to_ros(gz_msg.normal(i), ros_normal);
    ros_msg.normals.push_back(ros_normal);
  }
  for (auto i = 0; i < gz_msg.depth_size(); ++i) {
    ros_msg.depths.push_back(gz_msg.depth(i));
  }
  for (auto i = 0; i < gz_msg.wrench_size(); ++i) {
    ros_gz_interfaces::msg::JointWrench ros_joint_wrench;
    convert_gz_to_ros(gz_msg.wrench(i), ros_joint_wrench);
    ros_msg.wrenches.push_back(ros_joint_wrench);
  }
}

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Contacts & ros_msg,
  gz::msgs::Contacts & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  gz_msg.clear_contact();
  for (const auto & ros_contact : ros_msg.contacts) {
    auto gz_contact = gz_msg.add_contact();
    convert_ros_to_gz(ros_contact, *gz_contact);
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Contacts & gz_msg,
  ros_gz_interfaces::msg::Contacts & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  for (auto i = 0; i < gz_msg.contact_size(); ++i) {
    ros_gz_interfaces::msg::Contact ros_contact;
    convert_gz_to_ros(gz_msg.contact(i), ros_contact);
    ros_msg.contacts.push_back(ros_contact);
  }
}

<<<<<<< HEAD
#if HAVE_DATAFRAME
=======
>>>>>>> 63b651a (Garden EOL (#662))
template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Dataframe & ros_msg,
  gz::msgs::Dataframe & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  auto * rssiPtr = gz_msg.mutable_header()->add_data();
  rssiPtr->set_key("rssi");
  rssiPtr->add_value(std::to_string(ros_msg.rssi));

  gz_msg.set_src_address(ros_msg.src_address);
  gz_msg.set_dst_address(ros_msg.dst_address);

  gz_msg.set_data(&(ros_msg.data[0]), ros_msg.data.size());
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Dataframe & gz_msg,
  ros_gz_interfaces::msg::Dataframe & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  ros_msg.src_address = gz_msg.src_address();
  ros_msg.dst_address = gz_msg.dst_address();

  const auto & header = gz_msg.header();
  for (auto i = 0; i < header.data_size(); ++i) {
    if (header.data(i).key() == "rssi" && header.data(i).value_size() > 0) {
      try {
        ros_msg.rssi = std::stod(header.data(i).value(0));
      } catch (const std::invalid_argument &) {
        std::cerr << "RSSI value is invalid (" <<
          header.data(i).value(0) << ")" << std::endl;
      } catch (const std::out_of_range &) {
        std::cerr << "RSSI value is out of range (" <<
          header.data(i).value(0) << ")" << std::endl;
      }
    }
  }

  ros_msg.data.resize(gz_msg.data().size());
  std::copy(
    gz_msg.data().begin(),
    gz_msg.data().begin() + gz_msg.data().size(),
    ros_msg.data.begin());
}
<<<<<<< HEAD
#endif  // HAVE_DATAFRAME
=======
>>>>>>> 63b651a (Garden EOL (#662))

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::GuiCamera & ros_msg,
  gz::msgs::GUICamera & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, *gz_msg.mutable_header());
  gz_msg.set_name(ros_msg.name);
  gz_msg.set_view_controller(ros_msg.view_controller);
  convert_ros_to_gz(ros_msg.pose, *gz_msg.mutable_pose());
  convert_ros_to_gz(ros_msg.track, *gz_msg.mutable_track());
  gz_msg.set_projection_type(ros_msg.projection_type);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::GUICamera & gz_msg,
  ros_gz_interfaces::msg::GuiCamera & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  ros_msg.name = gz_msg.name();
  ros_msg.view_controller = gz_msg.view_controller();
  convert_gz_to_ros(gz_msg.pose(), ros_msg.pose);
  convert_gz_to_ros(gz_msg.track(), ros_msg.track);
  ros_msg.projection_type = gz_msg.projection_type();
}

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Light & ros_msg,
  gz::msgs::Light & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));

  gz_msg.set_name(ros_msg.name);
  if (ros_msg.type == 0) {
    gz_msg.set_type(gz::msgs::Light_LightType::Light_LightType_POINT);
  } else if (ros_msg.type == 1) {
    gz_msg.set_type(gz::msgs::Light_LightType::Light_LightType_SPOT);
  } else if (ros_msg.type == 2) {
    gz_msg.set_type(
      gz::msgs::Light_LightType::Light_LightType_DIRECTIONAL);
  }

  convert_ros_to_gz(ros_msg.pose, *gz_msg.mutable_pose());
  convert_ros_to_gz(ros_msg.diffuse, *gz_msg.mutable_diffuse());
  convert_ros_to_gz(ros_msg.specular, *gz_msg.mutable_specular());
  gz_msg.set_attenuation_constant(ros_msg.attenuation_constant);
  gz_msg.set_attenuation_linear(ros_msg.attenuation_linear);
  gz_msg.set_attenuation_quadratic(ros_msg.attenuation_quadratic);
  convert_ros_to_gz(ros_msg.direction, *gz_msg.mutable_direction());
  gz_msg.set_range(ros_msg.range);
  gz_msg.set_cast_shadows(ros_msg.cast_shadows);
  gz_msg.set_spot_inner_angle(ros_msg.spot_inner_angle);
  gz_msg.set_spot_outer_angle(ros_msg.spot_outer_angle);
  gz_msg.set_spot_falloff(ros_msg.spot_falloff);

  gz_msg.set_id(ros_msg.id);

  gz_msg.set_parent_id(ros_msg.parent_id);

  gz_msg.set_intensity(ros_msg.intensity);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Light & gz_msg,
  ros_gz_interfaces::msg::Light & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  ros_msg.name = gz_msg.name();
  if (gz_msg.type() ==
    gz::msgs::Light_LightType::Light_LightType_POINT)
  {
    ros_msg.type = 0;
  } else if (gz_msg.type() == gz::msgs::Light_LightType::Light_LightType_SPOT) {
    ros_msg.type = 1;
  } else if (gz_msg.type() == gz::msgs::Light_LightType::Light_LightType_DIRECTIONAL) {
    ros_msg.type = 2;
  }

  convert_gz_to_ros(gz_msg.pose(), ros_msg.pose);
  convert_gz_to_ros(gz_msg.diffuse(), ros_msg.diffuse);
  convert_gz_to_ros(gz_msg.specular(), ros_msg.specular);
  ros_msg.attenuation_constant = gz_msg.attenuation_constant();
  ros_msg.attenuation_linear = gz_msg.attenuation_linear();
  ros_msg.attenuation_quadratic = gz_msg.attenuation_quadratic();
  convert_gz_to_ros(gz_msg.direction(), ros_msg.direction);
  ros_msg.range = gz_msg.range();
  ros_msg.cast_shadows = gz_msg.cast_shadows();
  ros_msg.spot_inner_angle = gz_msg.spot_inner_angle();
  ros_msg.spot_outer_angle = gz_msg.spot_outer_angle();
  ros_msg.spot_falloff = gz_msg.spot_falloff();

  ros_msg.id = gz_msg.id();

  ros_msg.parent_id = gz_msg.parent_id();

  ros_msg.intensity = gz_msg.intensity();
}

<<<<<<< HEAD
#if HAVE_MATERIALCOLOR
=======
>>>>>>> 63b651a (Garden EOL (#662))
template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::MaterialColor & ros_msg,
  gz::msgs::MaterialColor & gz_msg)
{
  using EntityMatch = gz::msgs::MaterialColor::EntityMatch;

  switch (ros_msg.entity_match) {
    case ros_gz_interfaces::msg::MaterialColor::FIRST:
      gz_msg.set_entity_match(EntityMatch::MaterialColor_EntityMatch_FIRST);
      break;
    case ros_gz_interfaces::msg::MaterialColor::ALL:
      gz_msg.set_entity_match(EntityMatch::MaterialColor_EntityMatch_ALL);
      break;
    default:
      std::cerr << "Unsupported entity match type ["
                << ros_msg.entity_match << "]\n";
  }

  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  convert_ros_to_gz(ros_msg.entity, *gz_msg.mutable_entity());
  convert_ros_to_gz(ros_msg.ambient, *gz_msg.mutable_ambient());
  convert_ros_to_gz(ros_msg.diffuse, *gz_msg.mutable_diffuse());
  convert_ros_to_gz(ros_msg.specular, *gz_msg.mutable_specular());
  convert_ros_to_gz(ros_msg.emissive, *gz_msg.mutable_emissive());

  gz_msg.set_shininess(ros_msg.shininess);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::MaterialColor & gz_msg,
  ros_gz_interfaces::msg::MaterialColor & ros_msg)
{
  using EntityMatch = gz::msgs::MaterialColor::EntityMatch;
  if (gz_msg.entity_match() == EntityMatch::MaterialColor_EntityMatch_FIRST) {
    ros_msg.entity_match = ros_gz_interfaces::msg::MaterialColor::FIRST;
/* *INDENT-OFF* */
  } else if (gz_msg.entity_match() ==
    EntityMatch::MaterialColor_EntityMatch_ALL) {
/* *INDENT-ON* */
    ros_msg.entity_match = ros_gz_interfaces::msg::MaterialColor::ALL;
  } else {
    std::cerr << "Unsupported EntityMatch [" <<
      gz_msg.entity_match() << "]" << std::endl;
  }
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  convert_gz_to_ros(gz_msg.entity(), ros_msg.entity);
  convert_gz_to_ros(gz_msg.ambient(), ros_msg.ambient);
  convert_gz_to_ros(gz_msg.diffuse(), ros_msg.diffuse);
  convert_gz_to_ros(gz_msg.specular(), ros_msg.specular);
  convert_gz_to_ros(gz_msg.emissive(), ros_msg.emissive);

  ros_msg.shininess = gz_msg.shininess();
}
<<<<<<< HEAD
#endif  // HAVE_MATERIALCOLOR
=======
>>>>>>> 63b651a (Garden EOL (#662))

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::SensorNoise & ros_msg,
  gz::msgs::SensorNoise & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, *gz_msg.mutable_header());
  if (ros_msg.type == 0) {
    gz_msg.set_type(gz::msgs::SensorNoise_Type::SensorNoise_Type_NONE);
  } else if (ros_msg.type == 2) {
    gz_msg.set_type(gz::msgs::SensorNoise_Type::SensorNoise_Type_GAUSSIAN);
  } else if (ros_msg.type == 3) {
    gz_msg.set_type(gz::msgs::SensorNoise_Type::SensorNoise_Type_GAUSSIAN_QUANTIZED);
  }

  gz_msg.set_mean(ros_msg.mean);
  gz_msg.set_stddev(ros_msg.stddev);
  gz_msg.set_bias_mean(ros_msg.bias_mean);
  gz_msg.set_bias_stddev(ros_msg.bias_stddev);
  gz_msg.set_precision(ros_msg.precision);
  gz_msg.set_dynamic_bias_stddev(ros_msg.dynamic_bias_stddev);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::SensorNoise & gz_msg,
  ros_gz_interfaces::msg::SensorNoise & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  if (gz_msg.type() == gz::msgs::SensorNoise_Type::SensorNoise_Type_NONE) {
    ros_msg.type = 0;
  } else if (gz_msg.type() == gz::msgs::SensorNoise_Type::SensorNoise_Type_GAUSSIAN) {
    ros_msg.type = 2;
<<<<<<< HEAD
  } else if (gz_msg.type() ==  // NOLINT
    gz::msgs::SensorNoise_Type::SensorNoise_Type_GAUSSIAN_QUANTIZED)  // NOLINT
  {  // NOLINT
=======
  } else if (gz_msg.type() == gz::msgs::SensorNoise_Type::SensorNoise_Type_GAUSSIAN_QUANTIZED) {
>>>>>>> 63b651a (Garden EOL (#662))
    ros_msg.type = 3;
  }

  ros_msg.mean = gz_msg.mean();
  ros_msg.stddev = gz_msg.stddev();
  ros_msg.bias_mean = gz_msg.bias_mean();
  ros_msg.bias_stddev = gz_msg.bias_stddev();
  ros_msg.precision = gz_msg.precision();
  ros_msg.dynamic_bias_stddev = gz_msg.dynamic_bias_stddev();
}

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::StringVec & ros_msg,
  gz::msgs::StringMsg_V & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, *gz_msg.mutable_header());
  for (const auto & elem : ros_msg.data) {
    auto * new_elem = gz_msg.add_data();
    *new_elem = elem;
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::StringMsg_V & gz_msg,
  ros_gz_interfaces::msg::StringVec & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  for (const auto & elem : gz_msg.data()) {
    ros_msg.data.emplace_back(elem);
  }
}

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::ParamVec & ros_msg,
  gz::msgs::Param & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));

  for (auto param : ros_msg.params) {
    gz::msgs::Any anyValue;
    convert_ros_to_gz(param.value, anyValue);
    auto new_param = gz_msg.mutable_params();
    (*new_param)[param.name] = anyValue;
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Param & gz_msg,
  ros_gz_interfaces::msg::ParamVec & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  for (auto it : gz_msg.params()) {
    rcl_interfaces::msg::Parameter p;
    p.name = it.first;
    convert_gz_to_ros(it.second, p.value);
    ros_msg.params.push_back(p);
  }

  for (int childIdx = 0; childIdx < gz_msg.children().size(); ++childIdx) {
    auto child = gz_msg.children().Get(childIdx);
    ros_gz_interfaces::msg::ParamVec child_vec;
    convert_gz_to_ros(child, child_vec);

    for (size_t entryIdx = 0; entryIdx < child_vec.params.size(); ++entryIdx) {
      auto ros_param = child_vec.params[entryIdx];
      ros_param.name = "child_" + std::to_string(childIdx) + "/" + ros_param.name;
      ros_msg.params.push_back(ros_param);
    }
  }
}

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::ParamVec & ros_msg,
  gz::msgs::Param_V & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));

  // This will store all of the parameters available in the ros_msg in the
  // first entry of the parameter vector
  // \TODO(mjcarroll) Make this work fully round trip
  // To do round trip, we must parse the parameter names and insert them
  // into the correct indicies in the parameter vector

  auto entry = gz_msg.mutable_param()->Add();
  convert_ros_to_gz(ros_msg, *entry);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Param_V & gz_msg,
  ros_gz_interfaces::msg::ParamVec & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  // Flatten the entries in the vector into a single range
  for (int paramIdx = 0; paramIdx < gz_msg.param().size(); ++paramIdx) {
    auto param = gz_msg.param().Get(paramIdx);
    ros_gz_interfaces::msg::ParamVec param_vec;
    convert_gz_to_ros(param, param_vec);

    for (size_t entryIdx = 0; entryIdx < param_vec.params.size(); ++entryIdx) {
      auto ros_param = param_vec.params[entryIdx];
      ros_param.name = "param_" + std::to_string(paramIdx) + "/" + ros_param.name;
      ros_msg.params.push_back(ros_param);
    }
  }
}

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::TrackVisual & ros_msg,
  gz::msgs::TrackVisual & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, *gz_msg.mutable_header());
  gz_msg.set_name(ros_msg.name);
  gz_msg.set_id(ros_msg.id);
  gz_msg.set_inherit_orientation(ros_msg.inherit_orientation);
  gz_msg.set_min_dist(ros_msg.min_dist);
  gz_msg.set_max_dist(ros_msg.max_dist);
  gz_msg.set_static_(ros_msg.is_static);
  gz_msg.set_use_model_frame(ros_msg.use_model_frame);
  convert_ros_to_gz(ros_msg.xyz, *gz_msg.mutable_xyz());
  gz_msg.set_inherit_yaw(ros_msg.inherit_yaw);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::TrackVisual & gz_msg,
  ros_gz_interfaces::msg::TrackVisual & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  ros_msg.name = gz_msg.name();
  ros_msg.id = gz_msg.id();
  ros_msg.inherit_orientation = gz_msg.inherit_orientation();
  ros_msg.min_dist = gz_msg.min_dist();
  ros_msg.max_dist = gz_msg.max_dist();
  ros_msg.is_static = gz_msg.static_();
  ros_msg.use_model_frame = gz_msg.use_model_frame();
  convert_gz_to_ros(gz_msg.xyz(), ros_msg.xyz);
  ros_msg.inherit_yaw = gz_msg.inherit_yaw();
}

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::VideoRecord & ros_msg,
  gz::msgs::VideoRecord & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, *gz_msg.mutable_header());
  gz_msg.set_start(ros_msg.start);
  gz_msg.set_stop(ros_msg.stop);
  gz_msg.set_format(ros_msg.format);
  gz_msg.set_save_filename(ros_msg.save_filename);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::VideoRecord & gz_msg,
  ros_gz_interfaces::msg::VideoRecord & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  ros_msg.start = gz_msg.start();
  ros_msg.stop = gz_msg.stop();
  ros_msg.format = gz_msg.format();
  ros_msg.save_filename = gz_msg.save_filename();
}

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::WorldControl & ros_msg,
  gz::msgs::WorldControl & gz_msg)
{
  gz_msg.set_pause(ros_msg.pause);
  gz_msg.set_step(ros_msg.step);
  gz_msg.set_multi_step(ros_msg.multi_step);
  convert_ros_to_gz(ros_msg.reset, *gz_msg.mutable_reset());
  gz_msg.set_seed(ros_msg.seed);
  convert_ros_to_gz(ros_msg.run_to_sim_time, *gz_msg.mutable_run_to_sim_time());
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::WorldControl & gz_msg,
  ros_gz_interfaces::msg::WorldControl & ros_msg)
{
  ros_msg.pause = gz_msg.pause();
  ros_msg.step = gz_msg.step();
  ros_msg.multi_step = gz_msg.multi_step();
  convert_gz_to_ros(gz_msg.reset(), ros_msg.reset);
  ros_msg.seed = gz_msg.seed();
  convert_gz_to_ros(gz_msg.run_to_sim_time(), ros_msg.run_to_sim_time);
}

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::WorldReset & ros_msg,
  gz::msgs::WorldReset & gz_msg)
{
  gz_msg.set_all(ros_msg.all);
  gz_msg.set_time_only(ros_msg.time_only);
  gz_msg.set_model_only(ros_msg.model_only);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::WorldReset & gz_msg,
  ros_gz_interfaces::msg::WorldReset & ros_msg)
{
  ros_msg.all = gz_msg.all();
  ros_msg.time_only = gz_msg.time_only();
  ros_msg.model_only = gz_msg.model_only();
}

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Float32Array & ros_msg,
  gz::msgs::Float_V & gz_msg)
{
  gz_msg.clear_data();
  for (auto const & t : ros_msg.data) {
    gz_msg.add_data(t);
  }
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Float_V & gz_msg,
  ros_gz_interfaces::msg::Float32Array & ros_msg)
{
  ros_msg.data.clear();
  for (auto const & p : gz_msg.data()) {
    ros_msg.data.push_back(p);
  }
}
}  // namespace ros_gz_bridge
