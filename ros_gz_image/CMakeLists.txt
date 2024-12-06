cmake_minimum_required(VERSION 3.5)

project(ros_gz_image)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

<<<<<<< HEAD
# TODO(CH3): Deprecated. Remove on tock.
if("$ENV{GZ_VERSION}" STREQUAL "" AND NOT "$ENV{IGNITION_VERSION}" STREQUAL "")
  message(DEPRECATION "Environment variable [IGNITION_VERSION] is deprecated. Use [GZ_VERSION] instead.")
  set(ENV{GZ_VERSION} $ENV{IGNITION_VERSION})
endif()

# Edifice
if("$ENV{GZ_VERSION}" STREQUAL "edifice")
  find_package(ignition-transport10 REQUIRED)
  set(GZ_TRANSPORT_VER ${ignition-transport10_VERSION_MAJOR})

  find_package(ignition-msgs7 REQUIRED)
  set(GZ_MSGS_VER ${ignition-msgs7_VERSION_MAJOR})

  set(GZ_TARGET_PREFIX ignition)

  message(STATUS "Compiling against Ignition Edifice")
# Fortress
elseif("$ENV{GZ_VERSION}" STREQUAL "garden")
  find_package(gz-transport12 REQUIRED)
  set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

  find_package(gz-msgs9 REQUIRED)
  set(GZ_MSGS_VER ${gz-msgs9_VERSION_MAJOR})

  set(GZ_TARGET_PREFIX gz)

  message(STATUS "Compiling against Gazebo Garden")
elseif("$ENV{GZ_VERSION}" STREQUAL "harmonic")
  find_package(gz-transport13 REQUIRED)
  find_package(gz-msgs10 REQUIRED)

  set(GZ_TARGET_PREFIX gz)
  set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})
  set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

  message(STATUS "Compiling against Gazebo Harmonic")
# Default to Fortress
else()
  find_package(ignition-transport11 REQUIRED)
  set(GZ_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

  find_package(ignition-msgs8 REQUIRED)
  set(GZ_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

  set(GZ_TARGET_PREFIX ignition)

  message(STATUS "Compiling against Ignition Fortress")
endif()
=======
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)

find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
>>>>>>> 63b651a (Garden EOL (#662))

include_directories(include)

set(executable
  image_bridge
)

add_executable(${executable}
  src/image_bridge.cpp
)

target_link_libraries(${executable}
<<<<<<< HEAD
  ${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}::core
  ${GZ_TARGET_PREFIX}-transport${GZ_TRANSPORT_VER}::core
=======
  gz-msgs::core
  gz-transport::core
>>>>>>> 63b651a (Garden EOL (#662))
)

ament_target_dependencies(${executable}
  "image_transport"
  "ros_gz_bridge"
  "rclcpp"
  "sensor_msgs"
)

install(TARGETS ${executable}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # find_package(rostest REQUIRED)
  #
  # set(test_publishers
  #   gz_publisher
  # )
  #
  # set(test_subscribers
  #   ros_subscriber
  # )
  #
  # foreach(test_publisher ${test_publishers})
  #   add_executable(${test_publisher}_image
  #     test/publishers/${test_publisher}.cpp
  #   )
  #   target_link_libraries(${test_publisher}_image
  #     ${catkin_LIBRARIES}
<<<<<<< HEAD
  #     ${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}::core
  #     ${GZ_TARGET_PREFIX}-transport${GZ_TRANSPORT_VER}::core
=======
  #     $gz-msgs$::core
  #     $gz-transport::core
>>>>>>> 63b651a (Garden EOL (#662))
  #     gtest
  #     gtest_main
  #   )
  # endforeach(test_publisher)
  #
  # foreach(test_subscriber ${test_subscribers})
  #   add_rostest_gtest(test_${test_subscriber}_image
  #     test/${test_subscriber}.test
  #     test/subscribers/${test_subscriber}.cpp)
  #   target_link_libraries(test_${test_subscriber}_image
  #     ${catkin_LIBRARIES}
<<<<<<< HEAD
  #     ${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}::core
  #     ${GZ_TARGET_PREFIX}-transport${GZ_TRANSPORT_VER}::core
=======
  #     $gz-msgs::core
  #     $gz-transport::core
>>>>>>> 63b651a (Garden EOL (#662))
  #   )
  # endforeach(test_subscriber)
endif()

ament_package()
