^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_gz_interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.4 (2025-02-12)
------------------

2.1.3 (2025-01-14)
------------------

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
* Add support for gz.msgs.EntityWrench (base branch: ros2) (`#573 <https://github.com/gazebosim/ros_gz//issues/573>`_)
* Add option to change material color from ROS. (`#521 <https://github.com/gazebosim/ros_gz//issues/521>`_)
  Forward port of `#486 <https://github.com/gazebosim/ros_gz//issues/486>`_.
  * Message and bridge for MaterialColor.
  This allows bridging MaterialColor from ROS to GZ and is
  important for allowing simulation users to create status lights.
  (cherry picked from commit 78dc4823121f085594e6028a93f1e571eb04f58b)
* Add option to change material color from ROS. (`#521 <https://github.com/gazebosim/ros_gz//issues/521>`_)
  Forward port of `#486 <https://github.com/gazebosim/ros_gz//issues/486>`_.
  * Message and bridge for MaterialColor.
  This allows bridging MaterialColor from ROS to GZ and is
  important for allowing simulation users to create status lights.
* Prepare for 1.0.0 Release (`#495 <https://github.com/gazebosim/ros_gz//issues/495>`_)
* 0.244.14
* Changelog
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
* 0.244.12
* Changelog
* 0.246.0
* Update changelogs
* SensorNoise msg bridging (`#417 <https://github.com/gazebosim/ros_gz//issues/417>`_)
* Added Altimeter msg bridging (`#413 <https://github.com/gazebosim/ros_gz//issues/413>`_)
* Port: humble to ros2 (`#386 <https://github.com/gazebosim/ros_gz//issues/386>`_)
* Merge branch 'humble' into mjcarroll/humble_to_ros2
* Update maintainers (`#376 <https://github.com/gazebosim/ros_gz//issues/376>`_)
* Humble ➡️ ROS2 (`#323 <https://github.com/gazebosim/ros_gz//issues/323>`_)
  Humble ➡️ ROS2
* Merge branch 'humble' into ports/humble_to_ros2
* Export rcl_interfaces exec dependency (`#317 <https://github.com/gazebosim/ros_gz//issues/317>`_)
* 0.245.0
* Changelog
* humble to ros2 (`#311 <https://github.com/gazebosim/ros_gz//issues/311>`_)
  Co-authored-by: Michael Carroll <michael@openrobotics.org>
* Merge remote-tracking branch 'origin/humble' into ahcorde/humble_to_ros2
* Contributors: Addisu Z. Taddese, Aditya Pande, Alejandro Hernández Cordero, Benjamin Perseghetti, Jose Luis Rivero, Michael Carroll, Victor T. Noppeney, ahcorde

1.0.0 (2024-04-24)
------------------

0.246.0 (2023-08-31)
--------------------
* SensorNoise msg bridging (`#417 <https://github.com/gazebosim/ros_gz/issues/417>`_)
* Added Altimeter msg bridging (`#413 <https://github.com/gazebosim/ros_gz/issues/413>`_)
* Port: humble to ros2 (`#386 <https://github.com/gazebosim/ros_gz/issues/386>`_)
* Merge branch 'humble' into mjcarroll/humble_to_ros2
* Update maintainers (`#376 <https://github.com/gazebosim/ros_gz/issues/376>`_)
* Humble ➡️ ROS2 (`#323 <https://github.com/gazebosim/ros_gz/issues/323>`_)
* Export rcl_interfaces exec dependency (`#317 <https://github.com/gazebosim/ros_gz/issues/317>`_)
* Contributors: Aditya Pande, Alejandro Hernández Cordero, Michael Carroll, ahcorde

0.245.0 (2022-10-12)
--------------------
* humble to ros2 (`#311 <https://github.com/gazebosim/ros_gz/issues/311>`_)
  Co-authored-by: Michael Carroll <michael@openrobotics.org>
* Merge remote-tracking branch 'origin/humble' into ahcorde/humble_to_ros2
* Contributors: Alejandro Hernández Cordero, ahcorde


0.244.10 (2023-05-03)
---------------------

0.244.9 (2022-11-03)
--------------------
* Export rcl_interfaces exec dependency (`#317 <https://github.com/gazebosim/ros_gz/issues/317>`_)
* Contributors: Michael Carroll

0.244.8 (2022-10-28)
--------------------

0.244.7 (2022-10-12)
--------------------
* Bridge between msgs::Float_V and ros_gz_interfaces/Float32Array msg types (`#306 <https://github.com/gazebosim/ros_gz/issues/306>`_)
  * bridge float_v and float32_multi_array msg type
  Co-authored-by: Ian Chen <ichen@openrobotics.org>
* Merge pull request `#275 <https://github.com/gazebosim/ros_gz/issues/275>`_ (Galactic to Humble)
  Galactic to Humble
* Merge branch 'ros2' into ports/galactic_to_ros2
* Contributors: Ian Chen, Michael Carroll

0.244.6 (2022-09-14)
--------------------

0.244.5 (2022-09-12)
--------------------
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
* [ros2] README updates (service bridge, Gazebo rename) (`#252 <https://github.com/gazebosim/ros_gz/issues/252>`_)
* Contributors: Louise Poubel

0.244.2 (2022-04-25)
--------------------
* [ros_gz_interfaces] Add GuiCamera, StringVec, TrackVisual, VideoRecord (`#214 <https://github.com/gazebosim/ros_gz/issues/214>`_)
  * [ros_gz_interfaces] Add more interface definitions.
  * Add converion functions for the added messages
  * Update the factory factory function with the new messages
  * Add new messages to docs
  * Add test cases for the new messages conversions
* Update maintainer for ros_gz_interfaces (`#204 <https://github.com/gazebosim/ros_gz/issues/204>`_)
* [ros2]  new package ros_gz_interfaces, provide some  Gazebo-specific ROS messages. (`#152 <https://github.com/gazebosim/ros_gz/issues/152>`_)
  * add new package ros_gz_interfaces,provide some Gazebo-specific ros .msg and .srv files
  * modify to match gz-msgs
  * add author info
  * modify comments
  * update code and doc style
* Contributors: Alejandro Hernández Cordero, Ivan Santiago Paunovic, Louise Poubel, Michael Carroll, ahcorde, gezp

0.244.1 (2022-01-04)
--------------------

0.244.0 (2021-12-30)
--------------------
* New Light Message, also bridge Color (`#187 <https://github.com/gazebosim/ros_gz/issues/187>`_)
* Expose Contacts through ROS bridge (`#175 <https://github.com/gazebosim/ros_gz/issues/175>`_)
* Contributors: Guillaume Doisy, Vatan Aksoy Tezer, William Lew

0.233.2 (2021-07-20)
--------------------
* [ros2]  new package ros_gz_interfaces, provide some  Gazebo-specific ROS messages. (`#152 <https://github.com/gazebosim/ros_gz/issues/152>`_)
  * add new package ros_gz_interfaces,provide some Gazebo-specific ros .msg and .srv files
  * modify to match gz-msgs
  * add author info
  * modify comments
  * update code and doc style
* Contributors: gezp
