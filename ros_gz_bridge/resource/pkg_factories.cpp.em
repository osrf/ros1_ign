// generated from ros_gz_bridge/resource/pkg_factories.cpp.em

@###############################################
@#
@# Factory template specializations based on
@# message types of a single ROS 2 package
@#
@# EmPy template for generating factories/<pkgname>.cpp
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - ros2_package_name (str)
@#    The ROS 2 package name of this file
@###############################################
@

#include "factories/@(ros2_package_name).hpp"

#include <memory>
#include <string>

#include "factory.hpp"
#include "ros_gz_bridge/convert/@(ros2_package_name).hpp"

namespace ros_gz_bridge
{

std::shared_ptr<FactoryInterface>
get_factory__@(ros2_package_name)(
  const std::string & ros_type_name,
  const std::string & gz_type_name)
{
@[for m in mappings]@
  if ((ros_type_name == "@(m.ros2_string())" || ros_type_name.empty()) &&
      (gz_type_name == "@(m.gz_string())" || gz_type_name == "@(m.ign_string())"))
  {
    return std::make_shared<
      Factory<
        @(m.ros2_type()),
<<<<<<< HEAD
        @(m.ign_type())
=======
        @(m.gz_type())
>>>>>>> 63b651a (Garden EOL (#662))
      >
    >("@(m.ros2_string())", "@(m.gz_string())");
  }
@[end for]@
  return nullptr;
}

@[for m in mappings]@
template<>
void
Factory<
  @(m.ros2_type()),
<<<<<<< HEAD
  @(m.ign_type())
>::convert_ros_to_gz(
  const @(m.ros2_type()) & ros_msg,
  @(m.ign_type()) & gz_msg)
=======
  @(m.gz_type())
>::convert_ros_to_gz(
  const @(m.ros2_type()) & ros_msg,
  @(m.gz_type()) & gz_msg)
>>>>>>> 63b651a (Garden EOL (#662))
{
  ros_gz_bridge::convert_ros_to_gz(ros_msg, gz_msg);
}

template<>
void
Factory<
  @(m.ros2_type()),
<<<<<<< HEAD
  @(m.ign_type())
>::convert_gz_to_ros(
  const @(m.ign_type()) & gz_msg,
=======
  @(m.gz_type())
>::convert_gz_to_ros(
  const @(m.gz_type()) & gz_msg,
>>>>>>> 63b651a (Garden EOL (#662))
  @(m.ros2_type()) & ros_msg)
{
  ros_gz_bridge::convert_gz_to_ros(gz_msg, ros_msg);
}
@[end for]@

}  // namespace ros_gz_bridge
