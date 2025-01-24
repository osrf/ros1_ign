^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_gz_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.3 (2025-01-14)
------------------
* Use both ROS package and message name for unique mappings (`#656 <https://github.com/gazebosim/ros_gz/issues/656>`_)
  Co-authored-by: Addisu Z. Taddese <addisu@openrobotics.org>
* Merge pull request `#664 <https://github.com/gazebosim/ros_gz/issues/664>`_ from azeey/improve_parameter_handling
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Merge pull request `#663 <https://github.com/gazebosim/ros_gz/issues/663>`_ from azeey/improve_arg_parsing
  The `RosGzBridge` and `GzServer` now support different spellings for
  boolean arguments (`True`, `true`). This also simplifies how
  conditionals are used to create composable nodes by evaluating the
  conditionals and using them as regular Python booleans instead of
  relying on `PythonExpression`. It was actually the `PythonExpression`
  that was preventing support of boolean arguments spelled `true`/`false`.
* Improve parameter handling for RosGzBridge
* Fix linter errors
* Improve argument parsing in Actions
  The `RosGzBridge` and `GzServer` now support different spellings for
  boolean arguments (`True`, `true`). This also simplifies how
  conditionals are used to create composable nodes by evaluating the
  conditionals and using them as regular Python booleans instead of
  relying on `PythonExpression`. It was actually the `PythonExpression`
  that was preventing support of boolean arguments spelled `true`/`false`.
* Fix use_respawn argument causing errors (`#651 <https://github.com/gazebosim/ros_gz/issues/651>`_)
* Add a way to pass extra parameters to ros_gz_bridge (`#628 <https://github.com/gazebosim/ros_gz/issues/628>`_)
  * Add bridge_params argument to ros_gz_bridge
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  Co-authored-by: Wiktor Bajor <69388767+Wiktor-99@users.noreply.github.com>
* Contributors: Aarav Gupta, Addisu Z. Taddese, Alejandro Hernández Cordero, Øystein Sture

2.1.2 (2024-10-31)
------------------

2.1.1 (2024-10-14)
------------------
* Extra parameter to start a container (`#616 <https://github.com/gazebosim/ros_gz/issues/616>`_)
* adds deadline and liveliness QoSPolicyKinds to qos_overriding_options (`#609 <https://github.com/gazebosim/ros_gz/issues/609>`_)
  Co-authored-by: nora <nko@bogaertsgl.com>
* Contributors: Carlos Agüero, norakon

2.1.0 (2024-09-12)
------------------
* Remove default_value for required arguments (`#602 <https://github.com/gazebosim/ros_gz//issues/602>`_)
  * Remove default_value for config_file
* Fix errors with name of bridge not being given (`#600 <https://github.com/gazebosim/ros_gz//issues/600>`_)
  * Add argument bridge_name to fix errors
* Use optional parameters in actions (`#601 <https://github.com/gazebosim/ros_gz//issues/601>`_)
* Contributors: Amronos, Carlos Agüero

2.0.1 (2024-08-29)
------------------
* Stamp all outgoing headers with the wall time if parameter override_timestamps_with_wall_time is set to true (`#562 <https://github.com/gazebosim/ros_gz/issues/562>`_)
* Contributors: Rein Appeldoorn

2.0.0 (2024-07-22)
------------------
* Making use_composition true by default (`#578 <https://github.com/gazebosim/ros_gz/issues/578>`_)
* Contributors: Addisu Z. Taddese

1.0.1 (2024-07-03)
------------------
* Add support for gz.msgs.EntityWrench (base branch: ros2) (`#573 <https://github.com/gazebosim/ros_gz//issues/573>`_)
* Merge pull request `#571 <https://github.com/gazebosim/ros_gz//issues/571>`_ from azeey/jazzy_to_ros2
  Merge jazzy ➡️  ros2
* Merge branch 'ros2' into jazzy_to_ros2
* Use memcpy instead of std::copy when bridging images (`#565 <https://github.com/gazebosim/ros_gz//issues/565>`_)
  While testing ros <-> gz communication using the bridge I noticed that the bridge was talking quite a bit of time copying images from Gazebo to ROS. I found that the std::copy operation that we're doing is substantially slower than the memcpy alternative. I think that in principle this shouldn't happen but the numbers are quite clear. Perhaps std::copy is doing something that doesn't use cache effectively
  ---------
  Co-authored-by: Jose Luis Rivero <jrivero@osrfoundation.org>
* Merge jazzy into ros2
* Merge pull request `#569 <https://github.com/gazebosim/ros_gz//issues/569>`_ from azeey/iron_to_jazzy
  Merge iron ➡️  jazzy
* Merge iron into jazzy
* Add option to change material color from ROS. (`#521 <https://github.com/gazebosim/ros_gz//issues/521>`_)
  Forward port of `#486 <https://github.com/gazebosim/ros_gz//issues/486>`_.
  * Message and bridge for MaterialColor.
  This allows bridging MaterialColor from ROS to GZ and is
  important for allowing simulation users to create status lights.
  (cherry picked from commit 78dc4823121f085594e6028a93f1e571eb04f58b)
* Merge pull request `#564 <https://github.com/gazebosim/ros_gz//issues/564>`_ from azeey/humble_to_iron
  Humble ➡️ Iron
* Merge humble -> iron
* Use `ignoreLocalMessages` in the bridge (`#559 <https://github.com/gazebosim/ros_gz//issues/559>`_)
  * Ignore local messages
* Update launch files with name parameter (`#556 <https://github.com/gazebosim/ros_gz//issues/556>`_)
  * Name is required.
* Ensure the same container is used for the bridge and gz_server (`#553 <https://github.com/gazebosim/ros_gz//issues/553>`_)
  This also adds a required `name` parameter for the bridge so that
  multiple different bridges can be created without name collision
* Launch ros_gz_bridge from xml (`#550 <https://github.com/gazebosim/ros_gz//issues/550>`_)
  * Add gzserver with ability to load an SDF file or string
* Launch gzserver and the bridge as composable nodes (`#528 <https://github.com/gazebosim/ros_gz//issues/528>`_)
  * Add gzserver with ability to load an SDF file or string
* Add option to change material color from ROS. (`#521 <https://github.com/gazebosim/ros_gz//issues/521>`_)
  Forward port of `#486 <https://github.com/gazebosim/ros_gz//issues/486>`_.
  * Message and bridge for MaterialColor.
  This allows bridging MaterialColor from ROS to GZ and is
  important for allowing simulation users to create status lights.
* populate imu covariances when converting (`#375 <https://github.com/gazebosim/ros_gz//issues/375>`_) (`#540 <https://github.com/gazebosim/ros_gz//issues/540>`_)
  Co-authored-by: El Jawad Alaa <ejalaa12@gmail.com>
* Prepare for 1.0.0 Release (`#495 <https://github.com/gazebosim/ros_gz//issues/495>`_)
* Use gz_vendor packages (`#531 <https://github.com/gazebosim/ros_gz//issues/531>`_)
* [backport Humble] Create bridge for GPSFix msg (`#316 <https://github.com/gazebosim/ros_gz//issues/316>`_) (`#538 <https://github.com/gazebosim/ros_gz//issues/538>`_)
  Co-authored-by: Rousseau Vincent <vincentrou@gmail.com>
* [backport Iron] Create bridge for GPSFix msg (`#316 <https://github.com/gazebosim/ros_gz//issues/316>`_) (`#537 <https://github.com/gazebosim/ros_gz//issues/537>`_)
  Co-authored-by: Rousseau Vincent <vincentrou@gmail.com>
* 0.244.14
* Changelog
* Added conversion for Detection3D and Detection3DArray (`#523 <https://github.com/gazebosim/ros_gz//issues/523>`_) (`#526 <https://github.com/gazebosim/ros_gz//issues/526>`_)
  Co-authored-by: wittenator <9154515+wittenator@users.noreply.github.com>
* Added conversion for Detection3D and Detection3DArray (`#523 <https://github.com/gazebosim/ros_gz//issues/523>`_) (`#525 <https://github.com/gazebosim/ros_gz//issues/525>`_)
  Co-authored-by: wittenator <9154515+wittenator@users.noreply.github.com>
* [Backport rolling] Add ROS namespaces to GZ topics (`#517 <https://github.com/gazebosim/ros_gz//issues/517>`_)
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
* ign to gz (`#519 <https://github.com/gazebosim/ros_gz//issues/519>`_)
* Add ROS namespaces to GZ topics (`#512 <https://github.com/gazebosim/ros_gz//issues/512>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Correctly export ros_gz_bridge for downstream targets (`#503 <https://github.com/gazebosim/ros_gz//issues/503>`_) (`#506 <https://github.com/gazebosim/ros_gz//issues/506>`_)
* Add a virtual destructor to suppress compiler warning (`#502 <https://github.com/gazebosim/ros_gz//issues/502>`_) (`#505 <https://github.com/gazebosim/ros_gz//issues/505>`_)
  Co-authored-by: Michael Carroll <mjcarroll@intrinsic.ai>
* Correctly export ros_gz_bridge for downstream targets (`#503 <https://github.com/gazebosim/ros_gz//issues/503>`_)
* Add a virtual destructor to suppress compiler warning (`#502 <https://github.com/gazebosim/ros_gz//issues/502>`_)
* Add option to change material color from ROS. (`#486 <https://github.com/gazebosim/ros_gz//issues/486>`_)
  * Message and bridge for MaterialColor.
  This allows bridging MaterialColor from ROS to GZ and is
  important for allowing simulation users to create status lights.
  ---------
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  Co-authored-by: Addisu Z. Taddese <addisuzt@intrinsic.ai>
  Co-authored-by: Addisu Z. Taddese <addisu@openrobotics.org>
* 0.244.13
* Changelog
* backport pr 374 (`#489 <https://github.com/gazebosim/ros_gz//issues/489>`_)
* populate imu covariances when converting (`#488 <https://github.com/gazebosim/ros_gz//issues/488>`_)
* 0.244.12
* Changelog
* Backport: Add conversion for geometry_msgs/msg/TwistStamped <-> gz.msgs.Twist (`#468 <https://github.com/gazebosim/ros_gz//issues/468>`_) (`#470 <https://github.com/gazebosim/ros_gz//issues/470>`_)
* Add conversion for geometry_msgs/msg/TwistStamped <-> gz.msgs.Twist (`#468 <https://github.com/gazebosim/ros_gz//issues/468>`_)
* Added messages for 2D Bounding Boxes to ros_gz_bridge (`#458 <https://github.com/gazebosim/ros_gz//issues/458>`_) (`#466 <https://github.com/gazebosim/ros_gz//issues/466>`_)
  Co-authored-by: Alejandro Hernandez Cordero <ahcorde@gmail.com>
* populate imu covariances when converting (`#375 <https://github.com/gazebosim/ros_gz//issues/375>`_)
* 0.246.0
* Update changelogs
* Add harmonic CI (`#447 <https://github.com/gazebosim/ros_gz//issues/447>`_)
  * Add harmonic CI
  * Include garden options
  * Add harmonic stanza
  * Additional message headers
  ---------
* SensorNoise msg bridging (`#417 <https://github.com/gazebosim/ros_gz//issues/417>`_)
* Added Altimeter msg bridging (`#413 <https://github.com/gazebosim/ros_gz//issues/413>`_)
* Update README.md (`#411 <https://github.com/gazebosim/ros_gz//issues/411>`_)
  The ROS type for gz.msgs.NavSat messages should be **sensor_msgs/msg/NavSatFix** instead of **sensor_msgs/msg/NavSatFixed**
* Add missing rosidl_cmake dep to ros_gz_bridge (`#391 <https://github.com/gazebosim/ros_gz//issues/391>`_)
  Co-authored-by: Chris Lalancette <clalancette@gmail.com>
* allow converting from/to TwistWithCovarianceStamped (`#374 <https://github.com/gazebosim/ros_gz//issues/374>`_)
  * allow converting from/to TwistWithCovarianceStamped
  --------
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Added doc (`#393 <https://github.com/gazebosim/ros_gz//issues/393>`_)
* Port: humble to ros2 (`#386 <https://github.com/gazebosim/ros_gz//issues/386>`_)
* Merge branch 'humble' into mjcarroll/humble_to_ros2
* allow converting from/to PoseWithCovarianceStamped (`#381 <https://github.com/gazebosim/ros_gz//issues/381>`_)
  * allow converting from/to PoseWithCovarianceStamped
* Add actuator_msgs to bridge. (`#378 <https://github.com/gazebosim/ros_gz//issues/378>`_)
* Update maintainers (`#376 <https://github.com/gazebosim/ros_gz//issues/376>`_)
* Fix warning message (`#371 <https://github.com/gazebosim/ros_gz//issues/371>`_)
* Improve error messages around config loading (`#356 <https://github.com/gazebosim/ros_gz//issues/356>`_)
* Bringing the Joy to gazebo. (`#350 <https://github.com/gazebosim/ros_gz//issues/350>`_)
  Enable using the gazebo bridge with Joy.
* Fix double wait in ros_gz_bridge (`#347 <https://github.com/gazebosim/ros_gz//issues/347>`_)
* Create bridge for GPSFix msg (`#316 <https://github.com/gazebosim/ros_gz//issues/316>`_)
* Humble ➡️ ROS2 (`#323 <https://github.com/gazebosim/ros_gz//issues/323>`_)
  Humble ➡️ ROS2
* Merge branch 'humble' into ports/humble_to_ros2
* 0.245.0
* Changelog
* humble to ros2 (`#311 <https://github.com/gazebosim/ros_gz//issues/311>`_)
  Co-authored-by: Michael Carroll <michael@openrobotics.org>
* Remove Humble+ deprecations (`#312 <https://github.com/gazebosim/ros_gz//issues/312>`_)
  * Remove Humble+ deprecations
* Merge remote-tracking branch 'origin/humble' into ahcorde/humble_to_ros2
* Remove all ignition references on ROS 2 branch (`#302 <https://github.com/gazebosim/ros_gz//issues/302>`_)
  * Remove all shims
  * Update CMakeLists and package.xml for garden
  * Complete garden gz renaming
  * Drop fortress CI
* Contributors: Addisu Z. Taddese, Aditya Pande, Alejandro Hernández Cordero, Arjun K Haridas, Benjamin Perseghetti, Carlos Agüero, El Jawad Alaa, Jose Luis Rivero, Krzysztof Wojciechowski, Michael Carroll, Rousseau Vincent, Victor T. Noppeney, Yadu, ahcorde, wittenator, ymd-stella

1.0.0 (2024-04-24)
------------------
* Use gz_vendor packages (`#531 <https://github.com/gazebosim/ros_gz/issues/531>`_)
* Added conversion for Detection3D and Detection3DArray (`#523 <https://github.com/gazebosim/ros_gz/issues/523>`_) (`#525 <https://github.com/gazebosim/ros_gz/issues/525>`_)
  Co-authored-by: wittenator <9154515+wittenator@users.noreply.github.com>
* [Backport rolling] Add ROS namespaces to GZ topics (`#517 <https://github.com/gazebosim/ros_gz/issues/517>`_)
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
* ign to gz (`#519 <https://github.com/gazebosim/ros_gz/issues/519>`_)
* Correctly export ros_gz_bridge for downstream targets (`#503 <https://github.com/gazebosim/ros_gz/issues/503>`_)
* Add a virtual destructor to suppress compiler warning (`#502 <https://github.com/gazebosim/ros_gz/issues/502>`_)
* Add conversion for geometry_msgs/msg/TwistStamped <-> gz.msgs.Twist (`#468 <https://github.com/gazebosim/ros_gz/issues/468>`_)
* Added messages for 2D Bounding Boxes to ros_gz_bridge (`#458 <https://github.com/gazebosim/ros_gz/issues/458>`_) (`#466 <https://github.com/gazebosim/ros_gz/issues/466>`_)
  Co-authored-by: Alejandro Hernandez Cordero <ahcorde@gmail.com>
* populate imu covariances when converting (`#375 <https://github.com/gazebosim/ros_gz/issues/375>`_)
* Contributors: Addisu Z. Taddese, Alejandro Hernández Cordero, El Jawad Alaa, Michael Carroll

0.246.0 (2023-08-31)
--------------------
* Add harmonic CI (`#447 <https://github.com/gazebosim/ros_gz/issues/447>`_)
  * Add harmonic CI
  * Include garden options
  * Add harmonic stanza
  * Additional message headers
  ---------
* SensorNoise msg bridging (`#417 <https://github.com/gazebosim/ros_gz/issues/417>`_)
* Added Altimeter msg bridging (`#413 <https://github.com/gazebosim/ros_gz/issues/413>`_)
* Update README.md (`#411 <https://github.com/gazebosim/ros_gz/issues/411>`_)
  The ROS type for gz.msgs.NavSat messages should be **sensor_msgs/msg/NavSatFix** instead of **sensor_msgs/msg/NavSatFixed**
* Add missing rosidl_cmake dep to ros_gz_bridge (`#391 <https://github.com/gazebosim/ros_gz/issues/391>`_)
  Co-authored-by: Chris Lalancette <clalancette@gmail.com>
* allow converting from/to TwistWithCovarianceStamped (`#374 <https://github.com/gazebosim/ros_gz/issues/374>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Added doc (`#393 <https://github.com/gazebosim/ros_gz/issues/393>`_)
* Port: humble to ros2 (`#386 <https://github.com/gazebosim/ros_gz/issues/386>`_)
* Merge branch 'humble' into mjcarroll/humble_to_ros2
* allow converting from/to PoseWithCovarianceStamped (`#381 <https://github.com/gazebosim/ros_gz/issues/381>`_)
  * allow converting from/to PoseWithCovarianceStamped
* Add actuator_msgs to bridge. (`#378 <https://github.com/gazebosim/ros_gz/issues/378>`_)
* Update maintainers (`#376 <https://github.com/gazebosim/ros_gz/issues/376>`_)
* Fix warning message (`#371 <https://github.com/gazebosim/ros_gz/issues/371>`_)
* Improve error messages around config loading (`#356 <https://github.com/gazebosim/ros_gz/issues/356>`_)
* Bringing the Joy to gazebo. (`#350 <https://github.com/gazebosim/ros_gz/issues/350>`_)
  Enable using the gazebo bridge with Joy.
* Fix double wait in ros_gz_bridge (`#347 <https://github.com/gazebosim/ros_gz/issues/347>`_)
* Create bridge for GPSFix msg (`#316 <https://github.com/gazebosim/ros_gz/issues/316>`_)
* Humble ➡️ ROS2 (`#323 <https://github.com/gazebosim/ros_gz/issues/323>`_)
* Contributors: Aditya Pande, Alejandro Hernández Cordero, Arjun K Haridas, Benjamin Perseghetti, El Jawad Alaa, Michael Carroll, Rousseau Vincent, Yadu, ahcorde, ymd-stella

0.245.0 (2022-10-12)
--------------------
* humble to ros2 (`#311 <https://github.com/gazebosim/ros_gz/issues/311>`_)
  Co-authored-by: Michael Carroll <michael@openrobotics.org>
* Remove Humble+ deprecations (`#312 <https://github.com/gazebosim/ros_gz/issues/312>`_)
  * Remove Humble+ deprecations
* Merge remote-tracking branch 'origin/humble' into ahcorde/humble_to_ros2
* Remove all ignition references on ROS 2 branch (`#302 <https://github.com/gazebosim/ros_gz/issues/302>`_)
  * Remove all shims
  * Update CMakeLists and package.xml for garden
  * Complete garden gz renaming
  * Drop fortress CI
* Contributors: Alejandro Hernández Cordero, Michael Carroll, ahcorde

0.244.10 (2023-05-03)
---------------------
* Fix warning message (`#371 <https://github.com/gazebosim/ros_gz/issues/371>`_)
* Introduce WrenchStamped into bridge (`#327 <https://github.com/gazebosim/ros_gz/issues/327>`_)
* Humbly bringing the Joy to gazebo. (`#353 <https://github.com/gazebosim/ros_gz/issues/353>`_)
* Make the bridge aware of both gz and ignition msgs (`#349 <https://github.com/gazebosim/ros_gz/issues/349>`_)
* Contributors: Benjamin Perseghetti, El Jawad Alaa, Michael Carroll, livanov93

0.244.9 (2022-11-03)
--------------------

0.244.8 (2022-10-28)
--------------------

0.244.7 (2022-10-12)
--------------------
* Make sure that ign\_* yaml configs work as well (`#310 <https://github.com/gazebosim/ros_gz/issues/310>`_)
* Bridge between msgs::Float_V and ros_gz_interfaces/Float32Array msg types (`#306 <https://github.com/gazebosim/ros_gz/issues/306>`_)
  * bridge float_v and float32_multi_array msg type
  Co-authored-by: Ian Chen <ichen@openrobotics.org>
* Bridge between msgs::Pose_V and geometry_msgs/PoseArray msg types (`#305 <https://github.com/gazebosim/ros_gz/issues/305>`_)
* replace ign with gz in ros_gz_bridge README (`#303 <https://github.com/gazebosim/ros_gz/issues/303>`_)
* Merge pull request `#275 <https://github.com/gazebosim/ros_gz/issues/275>`_ (Galactic to Humble)
  Galactic to Humble
* Fix merge
* Merge branch 'ros2' into ports/galactic_to_ros2
* Contributors: Ian Chen, Michael Carroll, Olivier Kermorgant

0.244.6 (2022-09-14)
--------------------

0.244.5 (2022-09-12)
--------------------
* Fix missing msgs include and packages.xml deps (`#292 <https://github.com/gazebosim/ros_gz/issues/292>`_)
  * Fix missing msgs include and packages.xml deps
  * Add additional conditions to support gz sim invocation
  * Fix cpplint
* Add missing GZ_VERSION ticktocks (`#289 <https://github.com/gazebosim/ros_gz/issues/289>`_)
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
* Contributors: methylDragon

0.244.3 (2022-05-19)
--------------------
* Feature: set QoS options to override durability (`#250 <https://github.com/gazebosim/ros_gz/issues/250>`_)
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* [ros2] README updates (service bridge, Gazebo rename) (`#252 <https://github.com/gazebosim/ros_gz/issues/252>`_)
* Fix linter tests (`#251 <https://github.com/gazebosim/ros_gz/issues/251>`_)
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Adds pose and twist with covariance messages bridging (`#222 <https://github.com/gazebosim/ros_gz/issues/222>`_)
  * Added pose, twist and odometry with covariance messages bridging
* Contributors: Aditya Pande, Daisuke Nishimatsu, Louise Poubel

0.244.2 (2022-04-25)
--------------------
* Support bridging services (`#211 <https://github.com/gazebosim/ros_gz/issues/211>`_)
* Added reminder to hit play to receive images. (`#237 <https://github.com/gazebosim/ros_gz/issues/237>`_)
* Updated `ign topic` commnds on README (`#221 <https://github.com/gazebosim/ros_gz/issues/221>`_)
* Add conversions for ros_gz_interfaces/WorldControl and builtin_interfaces/Time (`#216 <https://github.com/gazebosim/ros_gz/issues/216>`_)
* [ros_gz_interfaces] Add GuiCamera, StringVec, TrackVisual, VideoRecord (`#214 <https://github.com/gazebosim/ros_gz/issues/214>`_)
* Break apart ros_subscriber test translation unit (`#212 <https://github.com/gazebosim/ros_gz/issues/212>`_)
* Bring ros2 branch up-to-date with Rolling (`#213 <https://github.com/gazebosim/ros_gz/issues/213>`_)
* Add missing dependency on rclcpp (`#209 <https://github.com/gazebosim/ros_gz/issues/209>`_)
* Separate galactic branch from ros2 branch (`#201 <https://github.com/gazebosim/ros_gz/issues/201>`_)
* 🏁 Dome EOL (`#198 <https://github.com/gazebosim/ros_gz/issues/198>`_)
* Contributors: Aditya Pande, Ivan Santiago Paunovic, Joep Tool, Louise Poubel, Michael Carroll

0.244.1 (2022-01-04)
--------------------
* Improve modularity of ign/ros publisher tests (`#194 <https://github.com/gazebosim/ros_gz/issues/194>`_)
* Contributors: Michael Carroll

0.244.0 (2021-12-30)
--------------------
* Default to Fortress for Rolling (future Humble) (`#195 <https://github.com/gazebosim/ros_gz/issues/195>`_)
* [ros2] 🏁 Dome EOL (`#199 <https://github.com/gazebosim/ros_gz/issues/199>`_)
* New Light Message, also bridge Color (`#187 <https://github.com/gazebosim/ros_gz/issues/187>`_)
* Statically link each translation unit (`#193 <https://github.com/gazebosim/ros_gz/issues/193>`_)
* Break apart convert and factories translation unit (`#192 <https://github.com/gazebosim/ros_gz/issues/192>`_)
* Fixed ROS subscriber test in ros_gz_bridge (`#189 <https://github.com/gazebosim/ros_gz/issues/189>`_)
* Enable QoS overrides (`#181 <https://github.com/gazebosim/ros_gz/issues/181>`_)
* Fixed ros ign bridge documentation (`#178 <https://github.com/gazebosim/ros_gz/issues/178>`_)
* Expose Contacts through ROS bridge (`#175 <https://github.com/gazebosim/ros_gz/issues/175>`_)
* Contributors: Alejandro Hernández Cordero, Guillaume Doisy, Louise Poubel, Michael Carroll, Vatan Aksoy Tezer, William Lew

0.233.2 (2021-07-20)
--------------------
* [ros2] Update version docs, add Galactic and Fortress (`#164 <https://github.com/gazebosim/ros_gz/issues/164>`_)
* Contributors: Louise Poubel

0.233.1 (2021-04-16)
--------------------
* Default to Edifice for Rolling (`#150 <https://github.com/gazebosim/ros_gz/issues/150>`_)
* Ignore local publications for ROS 2 subscriber (`#146 <https://github.com/gazebosim/ros_gz/issues/146>`_)
  - Note: Does not work with all rmw implementations (e.g.: FastRTPS)
* Update documentation for installation instructions and bridge examples (`#142 <https://github.com/gazebosim/ros_gz/issues/142>`_)
* Edifice support (`#140 <https://github.com/gazebosim/ros_gz/issues/140>`_)
* Add JointTrajectory message conversion (`#121 <https://github.com/gazebosim/ros_gz/issues/121>`_)
  Conversion between
  - ignition::msgs::JointTrajectory
  - trajectory_msgs::msg::JointTrajectory
* Add TFMessage / Pose_V and Float64 / Double conversions (`#117 <https://github.com/gazebosim/ros_gz/issues/117>`_)
  Addresses issue `#116 <https://github.com/gazebosim/ros_gz/issues/116>`_
* Updated prereq & branch name (`#113 <https://github.com/gazebosim/ros_gz/issues/113>`_)
* Update releases (`#108 <https://github.com/gazebosim/ros_gz/issues/108>`_)
* Updated README.md (`#104 <https://github.com/gazebosim/ros_gz/issues/104>`_)
* Add support for Dome (`#103 <https://github.com/gazebosim/ros_gz/issues/103>`_)
* Contributors: Alejandro Hernández Cordero, Andrej Orsula, Florent Audonnet, Jenn, Louise Poubel, Luca Della Vedova

0.221.1 (2020-08-19)
--------------------
* Add pkg-config as a buildtool dependency (`#102 <https://github.com/gazebosim/ros_gz/issues/102>`_)
* Port ros_gz_bridge tests to ROS 2 (`#98 <https://github.com/gazebosim/ros_gz/issues/98>`_)
* Rename test_utils.hpp (`#98 <https://github.com/gazebosim/ros_gz/issues/98>`_)
* Contributors: Louise Poubel, ahcorde

0.221.0 (2020-07-23)
--------------------
* Install only what's necessary, rename builtin_interfaces (`#95 <https://github.com/gazebosim/ros_gz/issues/95>`_)
* Move headers to src, rename builtin_interfaces (`#95 <https://github.com/gazebosim/ros_gz/issues/95>`_)
* Integer support (`#91 <https://github.com/gazebosim/ros_gz/issues/91>`_)
  Adds Int32 to the bridge.
* [ros2] Fixed CI - Added Foxy (`#89 <https://github.com/gazebosim/ros_gz/issues/89>`_)
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Ignore ros-args in parameter bridge (`#65 <https://github.com/gazebosim/ros_gz/issues/65>`_)
* Update Dashing docs (`#62 <https://github.com/gazebosim/ros_gz/issues/62>`_)
* Update dependencies to Citadel (`#57 <https://github.com/gazebosim/ros_gz/issues/57>`_)
* [WIP] Port ign_ros_gazebo_demos to ROS2 (`#58 <https://github.com/gazebosim/ros_gz/issues/58>`_)
  Port ros_gz_image to ROS2
  Port ros_gz_sim_demos to ROS2
* Add support for std_msgs/Empty (`#53 <https://github.com/gazebosim/ros_gz/issues/53>`_)
* Add support for std_msgs/Bool (`#50 <https://github.com/gazebosim/ros_gz/issues/50>`_)
* [ros2] Port ros_gz_bridge to ROS2 (`#45 <https://github.com/gazebosim/ros_gz/issues/45>`_)
* Enable ROS2 CI for Dashing branch (`#43 <https://github.com/gazebosim/ros_gz/issues/43>`_)
* Make all API and comments ROS-version agnostic
* Rename packages and fix compilation + tests
* Move files ros1 -> ros
* Contributors: Addisu Taddese, Alejandro Hernández Cordero, Jose Luis Rivero, Louise Poubel, Luca Della Vedova, Michael Carroll, Mohamed Ahmed, Shivesh Khaitan, chapulina

0.7.0 (2019-08-15)
------------------
* Merge pull request `#38 <https://github.com/osrf/ros1_ign_bridge/issues/38>`_ from osrf/unidirectional
  Support unidirectional bridge topics
* More examples
* Merge pull request `#37 <https://github.com/osrf/ros1_ign_bridge/issues/37>`_ from osrf/debug
  Adding debug and error statements
* Switch to characters supported by ros
* Merge branch 'debug' into unidirectional
* More output, and rosconsole depend
* Support specification of bridge direction
* Adding debug and error statements
* Contributors: Nate Koenig

0.6.3 (2019-08-04)
------------------

0.6.2 (2019-08-04)
------------------

0.6.1 (2019-08-04)
------------------
* Update README.md
* Contributors: Carlos Agüero

0.6.0 (2019-08-02)
------------------
* Merge pull request `#33 <https://github.com/osrf/ros1_ign_bridge/issues/33>`_ from osrf/issue_31
  Fix issue `#31 <https://github.com/osrf/ros1_ign_bridge/issues/31>`_
* Image bridge using image_transport (`#34 <https://github.com/osrf/ros1_ign_bridge/issues/34>`_)
  * Image bridge using image_transport
  * tests for image
  * correct metapackage
  * tests with catkin
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
  * Revert changes from `#32 <https://github.com/osrf/ros1_ign_bridge/issues/32>`_
  Signed-off-by: Louise Poubel <louise@openrobotics.org>
* Use intra-process field from messageInfo.
* Contributors: Carlos Aguero, Nate Koenig, chapulina

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
* tests and reverse bridge for pointcloud
* Bridge point cloud packed
* Contributors: Nate Koenig

0.3.1 (2019-07-01)
------------------

0.3.0 (2019-06-28)
------------------
* 0.2.0
* Conversion between nav_msgs/Odometry and ignition::msgs::Odometry (`#22 <https://github.com/osrf/ros1_ign_bridge/issues/22>`_)
  * Conversion between nav_msgs/Odometry and ignition::msgs::Odometry.
  * Update documentation.
  * More time to run tests
  * Cleaning test_utils.
  * Remove explicit ROS dependencies for Travis.
  * diff drive demo with cmd_vel and odom
  * process child frame id
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
* Point clouds for RGBD cameras (`#17 <https://github.com/osrf/ros1_ign_bridge/issues/17>`_)
  * Beginning of point cloud package
  * Populating image data, but result is not correct. Must find out where's the source of the problem.
  * RGB -> BGR: why?
  * Cleanup code and example
  * pointcloud -> point_cloud
  * add keys - how was this working before?
  * install wget
  * well, we need ign-gz2 :sweat_smile:
  * README update
  * PR feedback
  * .travis/build: rosdep skip ignition keys (`#18 <https://github.com/osrf/ros1_ign_bridge/issues/18>`_)
  * .travis/build: rosdep skip ignition keys
  * Update build
* Move package to subfolder, add metapackage (`#16 <https://github.com/osrf/ros1_ign_bridge/issues/16>`_)
* Contributors: Carlos Agüero, Nate Koenig, chapulina

0.2.2 (2019-05-20)
------------------

0.2.1 (2019-05-11)
------------------

0.2.0 (2019-05-09)
------------------

0.1.0 (2019-03-20)
------------------
