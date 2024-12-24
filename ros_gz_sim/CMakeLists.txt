cmake_minimum_required(VERSION 3.5)

project(ros_gz_sim)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)

find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)

find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)

find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
# Needed in launch/gz_sim.launch.py.in
set(GZ_SIM_VER ${gz-sim_VERSION_MAJOR})

gz_find_package(gflags
    REQUIRED
    PKGCONFIG gflags)
find_package(std_msgs REQUIRED)

# Install the python module for this package
ament_python_install_package(${PROJECT_NAME})

add_executable(create src/create.cpp)
ament_target_dependencies(create
  rclcpp
  std_msgs
)
target_link_libraries(create
  gflags
  gz-math::core
  gz-msgs::core
  gz-transport::core
  rcpputils::rcpputils
)

add_executable(remove src/remove.cpp)
ament_target_dependencies(remove
  rclcpp
  std_msgs
)

target_link_libraries(remove
  gz-msgs::core
  gz-transport::core
)

add_library(${PROJECT_NAME} SHARED src/Stopwatch.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

add_library(gzserver_component SHARED src/gzserver.cpp)
rclcpp_components_register_nodes(gzserver_component "ros_gz_sim::GzServer")
ament_target_dependencies(gzserver_component
  rclcpp
  rclcpp_components
  std_msgs
)
target_link_libraries(gzserver_component
  gz-sim::core
)
ament_target_dependencies(gzserver_component std_msgs)
rclcpp_components_register_node(
  gzserver_component
  PLUGIN "ros_gz_sim::GzServer"
  EXECUTABLE gzserver
)

configure_file(
  launch/gz_sim.launch.py.in
  launch/gz_sim.launch.py.configured
  @ONLY
)
file(GENERATE
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/launch/gz_sim.launch.py"
  INPUT "${CMAKE_CURRENT_BINARY_DIR}/launch/gz_sim.launch.py.configured"
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/launch/gz_sim.launch.py"
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES
  "launch/gz_server.launch"
  "launch/gz_server.launch.py"
  "launch/gz_spawn_model.launch"
  "launch/gz_spawn_model.launch.py"
  "launch/ros_gz_sim.launch"
  "launch/ros_gz_sim.launch.py"
  "launch/ros_gz_spawn_model.launch.py"
  "launch/gz_remove_model.launch.py"
  DESTINATION share/${PROJECT_NAME}/launch
)

install(TARGETS
  create
  remove
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  gzserver
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_gzserver_component)
install(TARGETS gzserver_component
        EXPORT export_gzserver_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(${PROJECT_NAME})

ament_export_dependencies(rclcpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # GTest
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_find_gtest()

  ament_add_gtest_executable(test_stopwatch
    test/test_stopwatch.cpp
  )
  ament_add_gtest_executable(test_create
    test/test_create.cpp
  )

  ament_add_gtest_executable(test_remove
    test/test_remove.cpp
  )

  ament_target_dependencies(test_stopwatch rclcpp)

  target_include_directories(test_stopwatch PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(test_stopwatch
    ${PROJECT_NAME}
  )
  target_link_libraries(test_create
    gz-transport::core
  )

  target_link_libraries(test_remove
    gz-transport::core
  )

  install(
    TARGETS test_stopwatch test_create test_remove
    DESTINATION lib/${PROJECT_NAME}
  )
  ament_add_gtest_test(test_stopwatch)
  add_launch_test(test/test_create_node.launch.py TIMEOUT 200)
  add_launch_test(test/test_remove_node.launch.py TIMEOUT 200)
endif()

ament_package()
