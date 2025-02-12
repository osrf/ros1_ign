^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros1_gz_sim_demos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.4 (2025-02-12)
------------------

2.1.3 (2025-01-14)
------------------
* Refactor triggered_camera demo (`#645 <https://github.com/gazebosim/ros_gz/issues/645>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Refactor rgbd_camera_bridge demo (`#643 <https://github.com/gazebosim/ros_gz/issues/643>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Refactor diff_drive demo (`#635 <https://github.com/gazebosim/ros_gz/issues/635>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Refactor gpu_lidar_bridge demo (`#636 <https://github.com/gazebosim/ros_gz/issues/636>`_)
  * Refactor gpu_lidar_bridge demo
* Refactor camera demo (`#634 <https://github.com/gazebosim/ros_gz/issues/634>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Refactor battery demo (`#633 <https://github.com/gazebosim/ros_gz/issues/633>`_)
* Refactor tf_bridge demo (`#644 <https://github.com/gazebosim/ros_gz/issues/644>`_)
* Refactor magnetometer demo (`#638 <https://github.com/gazebosim/ros_gz/issues/638>`_)
* Refactor imu demo (`#637 <https://github.com/gazebosim/ros_gz/issues/637>`_)
  * Refactor imu demo
* Refactor navsat_gpxfix demo (`#642 <https://github.com/gazebosim/ros_gz/issues/642>`_)
  * Refactor navsat_gpxfix demo
* Refactor navsat demo (`#639 <https://github.com/gazebosim/ros_gz/issues/639>`_)
  * Refactor navsat demo
* Refactor air pressure demo (`#632 <https://github.com/gazebosim/ros_gz/issues/632>`_)
  * Refactor air pressure demo
  Co-authored-by: Addisu Z. Taddese <addisu@openrobotics.org>
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Contributors: Carlos Agüero

2.1.2 (2024-10-31)
------------------

2.1.1 (2024-10-14)
------------------

2.1.0 (2024-09-12)
------------------

2.0.1 (2024-08-29)
------------------

2.0.0 (2024-07-22)
------------------

1.0.1 (2024-07-03)
------------------
* Prepare for 1.0.0 Release (`#495 <https://github.com/gazebosim/ros_gz//issues/495>`_)
* Use gz_vendor packages (`#531 <https://github.com/gazebosim/ros_gz//issues/531>`_)
* [backport Humble] Create bridge for GPSFix msg (`#316 <https://github.com/gazebosim/ros_gz//issues/316>`_) (`#538 <https://github.com/gazebosim/ros_gz//issues/538>`_)
  Co-authored-by: Rousseau Vincent <vincentrou@gmail.com>
* [backport Iron] Create bridge for GPSFix msg (`#316 <https://github.com/gazebosim/ros_gz//issues/316>`_) (`#537 <https://github.com/gazebosim/ros_gz//issues/537>`_)
  Co-authored-by: Rousseau Vincent <vincentrou@gmail.com>
* 0.244.14
* Changelog
* 0.244.13
* Changelog
* Remove deprecations using ros_gz_sim_create (`#476 <https://github.com/gazebosim/ros_gz//issues/476>`_)
* 0.244.12
* Changelog
* 0.246.0
* Update changelogs
* Added more topic to the bridge (`#422 <https://github.com/gazebosim/ros_gz//issues/422>`_)
* Fix incorrect subscription on demo (`#405 <https://github.com/gazebosim/ros_gz//issues/405>`_) (`#408 <https://github.com/gazebosim/ros_gz//issues/408>`_)
  This PR fixes an incorrect subscription on one of the demos. Running
  ```
  ros2 launch ros_gz_sim_demos gpu_lidar_bridge.launch.py
  ```
  causes rviz2 to crash and exit with the error:
  ```
  rviz2-3]
  [rviz2-3] >>> [rcutils|error_handling.c:108] rcutils_set_error_state()
  [rviz2-3] This error state is being overwritten:
  [rviz2-3]
  [rviz2-3]   'create_subscription() called for existing topic name rt/lidar with incompatible type sensor_msgs::msg::dds\_::PointCloud2\_, at ./src/subscription.cpp:146, at ./src/rcl/subscription.c:108'
  [rviz2-3]
  [rviz2-3] with this new error message:
  [rviz2-3]
  [rviz2-3]   'invalid allocator, at ./src/rcl/subscription.c:218'
  [rviz2-3]
  [rviz2-3] rcutils_reset_error() should be called after error handling to avoid this.
  ```
  This is due to an incorrect subscription on the part of the demo. This
  PR fixes it by getting a subscription to the right topic for the
  pointcloud display. (`lidar/points` instead of `lidar`). Was tested on
  garden + humble.
  Co-authored-by: Arjo Chakravarty <arjoc@intrinsic.ai>
* Port: humble to ros2 (`#386 <https://github.com/gazebosim/ros_gz//issues/386>`_)
* Merge branch 'humble' into mjcarroll/humble_to_ros2
* Update maintainers (`#376 <https://github.com/gazebosim/ros_gz//issues/376>`_)
* Rename 'ign gazebo' to 'gz sim' (`#343 <https://github.com/gazebosim/ros_gz//issues/343>`_)
* Create bridge for GPSFix msg (`#316 <https://github.com/gazebosim/ros_gz//issues/316>`_)
* Humble ➡️ ROS2 (`#323 <https://github.com/gazebosim/ros_gz//issues/323>`_)
  Humble ➡️ ROS2
* Merge branch 'humble' into ports/humble_to_ros2
* Fixed ros_gz_sim_demos launch files (`#319 <https://github.com/gazebosim/ros_gz//issues/319>`_)
* 0.245.0
* Changelog
* humble to ros2 (`#311 <https://github.com/gazebosim/ros_gz//issues/311>`_)
  Co-authored-by: Michael Carroll <michael@openrobotics.org>
* Merge remote-tracking branch 'origin/humble' into ahcorde/humble_to_ros2
* Remove all ignition references on ROS 2 branch (`#302 <https://github.com/gazebosim/ros_gz//issues/302>`_)
  * Remove all shims
  * Update CMakeLists and package.xml for garden
  * Complete garden gz renaming
  * Drop fortress CI
* Contributors: Addisu Z. Taddese, Aditya Pande, Alejandro Hernández Cordero, Clyde McQueen, Jose Luis Rivero, Michael Carroll, Rousseau Vincent, ahcorde

1.0.0 (2024-04-24)
------------------
* Use gz_vendor packages (`#531 <https://github.com/gazebosim/ros_gz/issues/531>`_)
* Remove deprecations using ros_gz_sim_create (`#476 <https://github.com/gazebosim/ros_gz/issues/476>`_)
* Contributors: Addisu Z. Taddese, Alejandro Hernández Cordero

0.246.0 (2023-08-31)
--------------------
* Added more topic to the bridge (`#422 <https://github.com/gazebosim/ros_gz/issues/422>`_)
* Fix incorrect subscription on demo (`#405 <https://github.com/gazebosim/ros_gz/issues/405>`_) (`#408 <https://github.com/gazebosim/ros_gz/issues/408>`_)
  Co-authored-by: Arjo Chakravarty <arjoc@intrinsic.ai>
* Port: humble to ros2 (`#386 <https://github.com/gazebosim/ros_gz/issues/386>`_)
* Merge branch 'humble' into mjcarroll/humble_to_ros2
* Update maintainers (`#376 <https://github.com/gazebosim/ros_gz/issues/376>`_)
* Rename 'ign gazebo' to 'gz sim' (`#343 <https://github.com/gazebosim/ros_gz/issues/343>`_)
* Create bridge for GPSFix msg (`#316 <https://github.com/gazebosim/ros_gz/issues/316>`_)
* Humble ➡️ ROS2 (`#323 <https://github.com/gazebosim/ros_gz/issues/323>`_)
* Fixed ros_gz_sim_demos launch files (`#319 <https://github.com/gazebosim/ros_gz/issues/319>`_)
* Contributors: Aditya Pande, Alejandro Hernández Cordero, Clyde McQueen, Michael Carroll, Rousseau Vincent, ahcorde

0.245.0 (2022-10-12)
--------------------
* humble to ros2 (`#311 <https://github.com/gazebosim/ros_gz/issues/311>`_)
  Co-authored-by: Michael Carroll <michael@openrobotics.org>
* Merge remote-tracking branch 'origin/humble' into ahcorde/humble_to_ros2
* Remove all ignition references on ROS 2 branch (`#302 <https://github.com/gazebosim/ros_gz/issues/302>`_)
  * Remove all shims
  * Update CMakeLists and package.xml for garden
  * Complete garden gz renaming
  * Drop fortress CI
* Contributors: Alejandro Hernández Cordero, Michael Carroll, ahcorde

0.244.10 (2023-05-03)
---------------------

0.244.9 (2022-11-03)
--------------------

0.244.8 (2022-10-28)
--------------------
* Fixed ros_gz_sim_demos launch files (`#319 <https://github.com/gazebosim/ros_gz/issues/319>`_) (`#320 <https://github.com/gazebosim/ros_gz/issues/320>`_)
* Contributors: Alejandro Hernández Cordero

0.244.7 (2022-10-12)
--------------------
* Merge pull request `#275 <https://github.com/gazebosim/ros_gz/issues/275>`_ (Galactic to Humble)
  Galactic to Humble
* Merge branch 'ros2' into ports/galactic_to_ros2
* Contributors: Michael Carroll

0.244.6 (2022-09-14)
--------------------

0.244.5 (2022-09-12)
--------------------
* sdformat_urdf parser demo (`#265 <https://github.com/gazebosim/ros_gz/issues/265>`_)
  * parser compatible model and launch framework
  * added ground plane, common gz plugins, demo commands and cleaned install paths
  * unique collision names and cleared flake
  * updating model config
  * building parser from source
  * fix flake and update deb dependency for garden
  * Move packages and files to gz
  * feedback and ign->gz
  * Support ros_ign migration
  Clean up shared libraries, and tick-tock RosGzPointCloud
  Tick-tock launch args
  Hard-tock ign\_ in sources
  Migrate ign, ign\_, IGN\_ for sources, launch, and test files
  Migrate IGN_XXX_VER, IGN_T, header guards
  Migrate launchfile, launchfile args, and test source references
  Migrate ros_ign_XXX and gz_gazebo -> gz_sim
  Migrate ros_ign_XXX project names
  Migrate Ign, ign-, IGN_DEPS, ign-gazebo
  Migrate ignitionrobotics, ignitionrobotics/ros_ign, osrf/ros_ign
  Migrate ignition-version, IGNITION_VERSION, Ignition <LIB>, ros_ign_ci
  * renaming and flake
  * added ros commands
  * gz-version
  * feedback and ci trial
  * removing garden condition
  Co-authored-by: Michael Carroll <michael@openrobotics.org>
  Co-authored-by: methylDragon <methylDragon@gmail.com>
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Support ros_ign migration (`#282 <https://github.com/gazebosim/ros_gz/issues/282>`_)
  Clean up shared libraries, and tick-tock RosGzPointCloud
  Tick-tock launch args
  Hard-tock ign\_ in sources
  Migrate ign, ign\_, IGN\_ for sources, launch, and test files
  Migrate IGN_XXX_VER, IGN_T, header guards
  Migrate launchfile, launchfile args, and test source references
  Migrate ros_ign_XXX and gz_gazebo -> gz_sim
  Migrate ros_ign_XXX project names
  Migrate Ign, ign-, IGN_DEPS, ign-gazebo
  Migrate ignitionrobotics, ignitionrobotics/ros_ign, osrf/ros_ign
  Migrate ignition-version, IGNITION_VERSION, Ignition <LIB>, ros_ign_ci
* Move packages and files to gz (`#282 <https://github.com/gazebosim/ros_gz/issues/282>`_)
* Contributors: Dharini Dutia, methylDragon

0.244.3 (2022-05-19)
--------------------
* [ros2] README updates (service bridge, Gazebo rename) (`#252 <https://github.com/gazebosim/ros_gz/issues/252>`_)
* Fix linter tests (`#251 <https://github.com/gazebosim/ros_gz/issues/251>`_)
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Joint state publisher and tf bridging demo (`#244 <https://github.com/gazebosim/ros_gz/issues/244>`_)
  * Added joint state publisher and tf bridge demo
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Contributors: Aditya Pande, Daisuke Nishimatsu, Louise Poubel

0.244.2 (2022-04-25)
--------------------
* Camera trigger demo (`#223 <https://github.com/gazebosim/ros_gz/issues/223>`_)
* Separate galactic branch from ros2 branch (`#201 <https://github.com/gazebosim/ros_gz/issues/201>`_)
* 🏁 Dome EOL (`#198 <https://github.com/gazebosim/ros_gz/issues/198>`_)
* Joint states tutorial (`#156 <https://github.com/gazebosim/ros_gz/issues/156>`_)
  Adds an rrbot model to demos and shows the usage of joint_states plugin.
* Contributors: Louise Poubel, Michael Carroll, Vatan Aksoy Tezer, William Lew

0.244.1 (2022-01-04)
--------------------

0.244.0 (2021-12-30)
--------------------
* Default to Fortress for Rolling (future Humble) (`#195 <https://github.com/gazebosim/ros_gz/issues/195>`_)
* [ros2] 🏁 Dome EOL (`#199 <https://github.com/gazebosim/ros_gz/issues/199>`_)
* Enable QoS overrides (`#181 <https://github.com/gazebosim/ros_gz/issues/181>`_)
* Contributors: Guillaume Doisy, Louise Poubel

0.233.2 (2021-07-20)
--------------------
* [ros2] Add exec depend on xacro for demos (`#170 <https://github.com/gazebosim/ros_gz/issues/170>`_)
* [ros2] Update version docs, add Galactic and Fortress (`#164 <https://github.com/gazebosim/ros_gz/issues/164>`_)
* Joint states tutorial (`#156 <https://github.com/gazebosim/ros_gz/issues/156>`_)
  Adds an rrbot model to demos and shows the usage of joint_states plugin.
* Contributors: Louise Poubel, Vatan Aksoy Tezer

0.233.1 (2021-04-16)
--------------------
* Default to Edifice for Rolling (`#150 <https://github.com/gazebosim/ros_gz/issues/150>`_)
* Minor updates for demos (`#144 <https://github.com/gazebosim/ros_gz/issues/144>`_)
  * Re-enable air pressure demo
  - Resolves https://github.com/gazebosim/ros_gz/issues/78
  * Add RQt topic viewer to IMU demo
  * Add image_topic argument for image_bridge demo
  * Do not normalize depth image in RViz2
* Edifice support (`#140 <https://github.com/gazebosim/ros_gz/issues/140>`_)
* Add topic flag to create robot  (`#128 <https://github.com/gazebosim/ros_gz/issues/128>`_)
  Now it is possible to run ros_gz_sim create specifying a topic as
  source of the robot description
  Add a launch file starting a ignition gazebo world and spawn a sphere in it.
  Additionally a rviz2 interface is loaded to show that also Rviz can load
  the robot description
  The newly created demo introduce a dependency on the robot_state_publisher package
* [ros2] Update releases (`#108 <https://github.com/gazebosim/ros_gz/issues/108>`_)
* Contributors: Andrej Orsula, Louise Poubel, Valerio Magnago

0.221.1 (2020-08-19)
--------------------

0.221.0 (2020-07-23)
--------------------
* Updated launch file to use ros_gz_sim (`#82 <https://github.com/gazebosim/ros_gz/issues/82>`_)
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Use new ros_gz_sim package on ROS 2 demos (`#85 <https://github.com/gazebosim/ros_gz/issues/85>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* [WIP] Port ign_ros_gazebo_demos to ROS2 (`#58 <https://github.com/gazebosim/ros_gz/issues/58>`_)
  Port ros_gz_image to ROS2
  Port ros_gz_sim_demos to ROS2
* Enable ROS2 CI for Dashing branch (`#43 <https://github.com/gazebosim/ros_gz/issues/43>`_)
* Make all API and comments ROS-version agnostic
* Rename packages and fix compilation + tests
* Move files ros1 -> ros
* Contributors: Alejandro Hernández Cordero, Jose Luis Rivero, Louise Poubel, chapulina

0.7.0 (2019-08-15)
------------------

0.6.3 (2019-08-04)
------------------

0.6.2 (2019-08-04)
------------------

0.6.1 (2019-08-04)
------------------

0.6.0 (2019-08-02)
------------------
* Image bridge using image_transport (`#34 <https://github.com/osrf/ros1_ign_bridge/issues/34>`_)
  * Image bridge using image_transport
  * tests for image
  * correct metapackage
  * tests with catkin
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
  * Revert changes from `#32 <https://github.com/osrf/ros1_ign_bridge/issues/32>`_
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* Revert "Pointcloud bridge demo for depth camera"
  This reverts commit 094cd40f21aed734d59c204172ad5afd7a26c8d6.
* Pointcloud bridge demo for depth camera
* Contributors: Louise Poubel, chapulina

* 0.5.0
* Battery state (`#30 <https://github.com/osrf/ros1_ign_bridge/issues/30>`_)
* Packed demo (`#29 <https://github.com/osrf/ros1_ign_bridge/issues/29>`_)
  * adding demo for point cloud packed bridge
  * correct rviz file
  * RGBD bridged cloud demo
* Merge pull request `#28 <https://github.com/osrf/ros1_ign_bridge/issues/28>`_ from osrf/pointcloudpacked
  Bridge point cloud packed
* Contributors: Nate Koenig, chapulina

* Battery state (`#30 <https://github.com/osrf/ros1_ign_bridge/issues/30>`_)
* Packed demo (`#29 <https://github.com/osrf/ros1_ign_bridge/issues/29>`_)
  * adding demo for point cloud packed bridge
  * correct rviz file
  * RGBD bridged cloud demo
* Merge pull request `#28 <https://github.com/osrf/ros1_ign_bridge/issues/28>`_ from osrf/pointcloudpacked
  Bridge point cloud packed
* Contributors: Nate Koenig, chapulina

0.4.0 (2019-07-16)
------------------

0.3.1 (2019-07-01)
------------------
* Merge pull request `#24 <https://github.com/osrf/ros1_ign_bridge/issues/24>`_ from osrf/fix_dep
  ignition-gazebo2 needed at build time
* ignition-gazebo2 needed at build time
* Contributors: Jose Luis Rivero

0.3.0 (2019-06-28)
------------------
* 0.2.0
* Merge pull request `#21 <https://github.com/osrf/ros1_ign_bridge/issues/21>`_ from osrf/lidar
  Point clouds from lidars
* Conversion between nav_msgs/Odometry and ignition::msgs::Odometry (`#22 <https://github.com/osrf/ros1_ign_bridge/issues/22>`_)
  * Conversion between nav_msgs/Odometry and ignition::msgs::Odometry.
  * Update documentation.
  * More time to run tests
  * Cleaning test_utils.
  * Remove explicit ROS dependencies for Travis.
  * diff drive demo with cmd_vel and odom
  * process child frame id
* final tweaks
* PC2 for gpu_lidar, 1 vertical sample
* Start of lidar PC
* Fluid pressure (`#20 <https://github.com/osrf/ros1_ign_bridge/issues/20>`_)
  * screenshots
  * missing IMU
  * Fluid pressure
  * Fix tests.
* Demos package (`#19 <https://github.com/osrf/ros1_ign_bridge/issues/19>`_)
  * Start of demos package: camera
  * IMU
  * depth camera
  * magnetometer
  * lidar, base launch
  * READMEs, RGBD camera
  * screenshots
  * missing IMU
  * set plugin path env
  * It's best to always set it
* Contributors: Carlos Agüero, Nate Koenig, chapulina

0.2.2 (2019-05-20)
------------------

0.2.1 (2019-05-11)
------------------

0.2.0 (2019-05-09)
------------------

0.1.0 (2019-03-20)
------------------
