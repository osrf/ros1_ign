^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_gz_sim
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.9 (2025-02-12)
------------------
* Fix spelling in entity creation (`#688 <https://github.com/gazebosim/ros_gz/issues/688>`_) (`#689 <https://github.com/gazebosim/ros_gz/issues/689>`_)
  (cherry picked from commit 5e3b0730359a4f3f23cb26f6083ae5620b9a5ea1)
  Co-authored-by: Leander Stephen D'Souza <leanderdsouza1234@gmail.com>
* Contributors: mergify[bot]

1.0.8 (2025-01-14)
------------------
* Shutdown explicitly while existing (`#623 <https://github.com/gazebosim/ros_gz/issues/623>`_) (`#679 <https://github.com/gazebosim/ros_gz/issues/679>`_)
  (cherry picked from commit 550ef7bd800c0dcc32ec7813168ef653ed169c89)
  Co-authored-by: ChenYing Kuo (CY) <evshary@gmail.com>
* Merge pull request `#670 <https://github.com/gazebosim/ros_gz/issues/670>`_ from gazebosim/ahcorde/jazzy/bp/663
  [backport Jazzy] Improve argument parsing in Actions (`#663 <https://github.com/gazebosim/ros_gz/issues/663>`_)
* Fix linter errors
* Improve argument parsing in Actions
  The `RosGzBridge` and `GzServer` now support different spellings for
  boolean arguments (`True`, `true`). This also simplifies how
  conditionals are used to create composable nodes by evaluating the
  conditionals and using them as regular Python booleans instead of
  relying on `PythonExpression`. It was actually the `PythonExpression`
  that was preventing support of boolean arguments spelled `true`/`false`.
* Set env path (`#659 <https://github.com/gazebosim/ros_gz/issues/659>`_) (`#661 <https://github.com/gazebosim/ros_gz/issues/661>`_)
  (cherry picked from commit 0eae6ffbaabb514c9a80e6add1f49f16ade72a2c)
  Co-authored-by: Tatsuro Sakaguchi <tacchan.mello.ioiq@gmail.com>
* Use member variables instead. (`#653 <https://github.com/gazebosim/ros_gz/issues/653>`_) (`#655 <https://github.com/gazebosim/ros_gz/issues/655>`_)
  (cherry picked from commit 5c1251a658ad8dd276664c2c88300d27572e25da)
  Co-authored-by: Carlos Agüero <caguero@openrobotics.org>
* Move gzserver logic to its action (`#646 <https://github.com/gazebosim/ros_gz/issues/646>`_) (`#649 <https://github.com/gazebosim/ros_gz/issues/649>`_)
  (cherry picked from commit 6bbb9e405856e23f445f9f8fa87f27ccbff4d114)
  Co-authored-by: Carlos Agüero <caguero@openrobotics.org>
* Add a way to pass extra parameters to ros_gz_bridge (`#628 <https://github.com/gazebosim/ros_gz/issues/628>`_) (`#648 <https://github.com/gazebosim/ros_gz/issues/648>`_)
  * Add bridge_params argument to ros_gz_bridge
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  Co-authored-by: Wiktor Bajor <69388767+Wiktor-99@users.noreply.github.com>
  (cherry picked from commit 558a1cfd55f9921e78a87c563d8ed847e9eae6bd)
  Co-authored-by: Aarav Gupta <amronos275@gmail.com>
* Add remove entity node (`#629 <https://github.com/gazebosim/ros_gz/issues/629>`_) (`#647 <https://github.com/gazebosim/ros_gz/issues/647>`_)
  (cherry picked from commit 04446e08e5a1153043e074528a7a3f2ec55449ab)
  Co-authored-by: Wiktor Bajor <69388767+Wiktor-99@users.noreply.github.com>
* Contributors: Addisu Z. Taddese, Alejandro Hernández Cordero, mergify[bot]

1.0.7 (2024-11-08)
------------------
* Bugfix: `if "false"` is always `True` (`#617 <https://github.com/gazebosim/ros_gz/issues/617>`_) (`#640 <https://github.com/gazebosim/ros_gz/issues/640>`_)
  There is an issue in this launch file when passing the string 'false' as
  an argument. In Python, non-empty strings are always evaluated as True,
  regardless of their content. This means that even if you pass 'false',
  the system will still evaluate it as True.
  This bug results in the launch system incorrectly calling the OnShutdown
  method twice. When any ROS launch action invokes a RosAdapter, it
  triggers the following exception: "Cannot shutdown a ROS adapter that is
  not running."
  To temporarily work around this issue, you can launch gz_sim_launch.py
  with the on_exit_shutdown argument set to an empty string. This prevents
  the erroneous shutdown sequence and avoids the associated exception.
  (cherry picked from commit 1e30af0105058d68c8f1c98f37904505f613cf97)
  Co-authored-by: Ignacio Vizzo <ignaciovizzo@gmail.com>
* Contributors: mergify[bot]

1.0.6 (2024-10-31)
------------------
* Create ros_gz_spawn_model.launch (`#604 <https://github.com/gazebosim/ros_gz/issues/604>`_) (`#627 <https://github.com/gazebosim/ros_gz/issues/627>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  (cherry picked from commit 02faa5b2383df27aa36321766c940924febdc5c6)
  Co-authored-by: Aarav Gupta <amronos275@gmail.com>
* Add create_own_container argument to ros_gz_spawn_model.launch.py (`#622 <https://github.com/gazebosim/ros_gz/issues/622>`_) (`#625 <https://github.com/gazebosim/ros_gz/issues/625>`_)
  (cherry picked from commit e8a3ec5404c71e75468d6ebf233d9cab09715b36)
  Co-authored-by: Amronos <134804732+Amronos@users.noreply.github.com>
* Fix ros_gz_sim.launch.py when create_own_container is enabled. (`#620 <https://github.com/gazebosim/ros_gz/issues/620>`_) (`#621 <https://github.com/gazebosim/ros_gz/issues/621>`_)
  (cherry picked from commit a4d00a90a1a5f753a0d0fbfe8e1e142540aebadd)
  Co-authored-by: Carlos Agüero <caguero@openrobotics.org>
* Extra parameter to start a container (`#616 <https://github.com/gazebosim/ros_gz/issues/616>`_) (`#618 <https://github.com/gazebosim/ros_gz/issues/618>`_)
  (cherry picked from commit 8115ccaaedea718841367eb64e500e13df392fd7)
  Co-authored-by: Carlos Agüero <caguero@openrobotics.org>
* Contributors: mergify[bot]

1.0.5 (2024-10-14)
------------------
* Merge pull request `#607 <https://github.com/gazebosim/ros_gz/issues/607>`_ from Amronos/ros2-jazzy-backport
* Fix changelogs and versions
* Name gazebo sim node (`#611 <https://github.com/gazebosim/ros_gz/issues/611>`_) (`#612 <https://github.com/gazebosim/ros_gz/issues/612>`_)
* Change world_string to model_string in gz_spawn_model files (`#606 <https://github.com/gazebosim/ros_gz/issues/606>`_)
* Use model string in ros_gz_spawn_model.launch.py (`#605 <https://github.com/gazebosim/ros_gz/issues/605>`_)
* Remove default_value for required arguments (`#602 <https://github.com/gazebosim/ros_gz/issues/602>`_)
* Fix errors with name of bridge not being given (`#600 <https://github.com/gazebosim/ros_gz/issues/600>`_)
* Restore launch file (`#603 <https://github.com/gazebosim/ros_gz/issues/603>`_)
* Use optional parameters in actions (`#601 <https://github.com/gazebosim/ros_gz/issues/601>`_)
* Wait for create service to be available. (`#588 <https://github.com/gazebosim/ros_gz/issues/588>`_)
* Update launch files with name parameter (`#556 <https://github.com/gazebosim/ros_gz/issues/556>`_)
* Launch gz_spawn_model from xml (`#551 <https://github.com/gazebosim/ros_gz/issues/551>`_)
* Launch ros_gz_bridge from xml (`#550 <https://github.com/gazebosim/ros_gz/issues/550>`_)
* Launch gzserver and the bridge as composable nodes (`#528 <https://github.com/gazebosim/ros_gz/issues/528>`_)
* Name gazebo sim node (`#611 <https://github.com/gazebosim/ros_gz/issues/611>`_) (`#612 <https://github.com/gazebosim/ros_gz/issues/612>`_)
* Contributors: Aarav Gupta, Addisu Z. Taddese, Alejandro Hernández Cordero, Amronos, Carlos Agüero, Sebastian Kasperski, mergify[bot]

1.0.4 (2024-08-29)
------------------

1.0.3 (2024-07-22)
------------------

1.0.2 (2024-07-03)
------------------
* Merge pull request `#569 <https://github.com/gazebosim/ros_gz//issues/569>`_ from azeey/iron_to_jazzy
  Merge iron ➡️  jazzy
* Merge remote-tracking branch 'origin/jazzy' into iron_to_jazzy
* Add a ROS node that runs Gazebo (`#500 <https://github.com/gazebosim/ros_gz//issues/500>`_) (`#567 <https://github.com/gazebosim/ros_gz//issues/567>`_)
  * Add gzserver with ability to load an SDF file or string
  ---------
  (cherry picked from commit 92a2891f4adf35e4a4119aca2447dee93e22a06a)
  Co-authored-by: Addisu Z. Taddese <addisu@openrobotics.org>
* Merge iron into jazzy
* Merge pull request `#564 <https://github.com/gazebosim/ros_gz//issues/564>`_ from azeey/humble_to_iron
  Humble ➡️ Iron
* Merge humble -> iron
* Prepare for 1.0.0 Release (`#495 <https://github.com/gazebosim/ros_gz//issues/495>`_)
* Use gz_vendor packages (`#531 <https://github.com/gazebosim/ros_gz//issues/531>`_)
* 0.244.14
* Changelog
* ign to gz (`#519 <https://github.com/gazebosim/ros_gz//issues/519>`_)
* Support `<gazebo_ros>` in `package.xml` exports (`#492 <https://github.com/gazebosim/ros_gz//issues/492>`_)
  This copies the implementation from `gazebo_ros_paths.py` to provide a
  way for packages to set resource paths from `package.xml`.
  ```
  e.g.  <export>
  <gazebo_ros gazebo_model_path="${prefix}/models"/>
  <gazebo_ros gazebo_media_path="${prefix}/models"/>
  </export>
  ```
  The value of `gazebo_model_path` and `gazebo_media_path` is appended to `GZ_SIM_RESOURCE_PATH`
  The value of `plugin_path` appended to `GZ_SIM_SYSTEM_PLUGIN_PATH`
  ---------
* Undeprecate use of commandline flags (`#491 <https://github.com/gazebosim/ros_gz//issues/491>`_)
* 0.244.13
* Changelog
* Remove deprecations using ros_gz_sim_create (`#476 <https://github.com/gazebosim/ros_gz//issues/476>`_)
* Added support for using ROS 2 parameters to spawn entities in Gazebo using ros_gz_sim::create (`#475 <https://github.com/gazebosim/ros_gz//issues/475>`_)
* Fix bug in `create` where command line arguments were truncated (`#472 <https://github.com/gazebosim/ros_gz//issues/472>`_)
* 0.244.12
* Changelog
* Filter ROS arguments before gflags parsing (`#453 <https://github.com/gazebosim/ros_gz//issues/453>`_)
* 0.246.0
* Update changelogs
* Add harmonic CI (`#447 <https://github.com/gazebosim/ros_gz//issues/447>`_)
  * Add harmonic CI
  * Include garden options
  * Add harmonic stanza
  * Additional message headers
  ---------
* Replace deprecated ign_find_package with gz_find_package (`#432 <https://github.com/gazebosim/ros_gz//issues/432>`_)
  Co-authored-by: jmackay2 <jmackay@gmail.com>
* Port: humble to ros2 (`#386 <https://github.com/gazebosim/ros_gz//issues/386>`_)
* Merge branch 'humble' into mjcarroll/humble_to_ros2
* Update maintainers (`#376 <https://github.com/gazebosim/ros_gz//issues/376>`_)
* set on_exit_shutdown argument for gz-sim ExecuteProcess (`#355 <https://github.com/gazebosim/ros_gz//issues/355>`_)
* Humble ➡️ ROS2 (`#323 <https://github.com/gazebosim/ros_gz//issues/323>`_)
  Humble ➡️ ROS2
* Merge branch 'humble' into ports/humble_to_ros2
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
* Contributors: Addisu Z. Taddese, Aditya Pande, Alejandro Hernández Cordero, Ayush Singh, Jose Luis Rivero, Michael Carroll, ahcorde, andermi, jmackay2, mergify[bot]

1.0.0 (2024-04-24)
------------------
* Use gz_vendor packages (`#531 <https://github.com/gazebosim/ros_gz/issues/531>`_)
* ign to gz (`#519 <https://github.com/gazebosim/ros_gz/issues/519>`_)
* Undeprecate use of commandline flags (`#491 <https://github.com/gazebosim/ros_gz/issues/491>`_)
* Remove deprecations using ros_gz_sim_create (`#476 <https://github.com/gazebosim/ros_gz/issues/476>`_)
* Added support for using ROS 2 parameters to spawn entities in Gazebo using ros_gz_sim::create (`#475 <https://github.com/gazebosim/ros_gz/issues/475>`_)
* Fix bug in `create` where command line arguments were truncated (`#472 <https://github.com/gazebosim/ros_gz/issues/472>`_)
* Filter ROS arguments before gflags parsing (`#453 <https://github.com/gazebosim/ros_gz/issues/453>`_)
* Contributors: Addisu Z. Taddese, Alejandro Hernández Cordero, Ayush Singh, Michael Carroll

0.246.0 (2023-08-31)
--------------------
* Add harmonic CI (`#447 <https://github.com/gazebosim/ros_gz/issues/447>`_)
  * Add harmonic CI
  * Include garden options
  * Add harmonic stanza
  * Additional message headers
  ---------
* Replace deprecated ign_find_package with gz_find_package (`#432 <https://github.com/gazebosim/ros_gz/issues/432>`_)
  Co-authored-by: jmackay2 <jmackay@gmail.com>
* Port: humble to ros2 (`#386 <https://github.com/gazebosim/ros_gz/issues/386>`_)
* Merge branch 'humble' into mjcarroll/humble_to_ros2
* Update maintainers (`#376 <https://github.com/gazebosim/ros_gz/issues/376>`_)
* set on_exit_shutdown argument for gz-sim ExecuteProcess (`#355 <https://github.com/gazebosim/ros_gz/issues/355>`_)
* Humble ➡️ ROS2 (`#323 <https://github.com/gazebosim/ros_gz/issues/323>`_)
* Remove all ignition references on ROS 2 branch (`#302 <https://github.com/gazebosim/ros_gz/issues/302>`_)
  * Remove all shims
  * Update CMakeLists and package.xml for garden
  * Complete garden gz renaming
  * Drop fortress CI
* Contributors: Aditya Pande, Alejandro Hernández Cordero, Michael Carroll, ahcorde, andermi, jmackay2

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
* Export ROS Stopwatch library (`#299 <https://github.com/gazebosim/ros_gz/issues/299>`_) (`#322 <https://github.com/gazebosim/ros_gz/issues/322>`_)
  New Stopwatch library needs to be exported and built as shared
  Co-authored-by: Michael Anderson <anderson@mbari.org>
* Contributors: Michael Carroll

0.244.8 (2022-10-28)
--------------------

0.244.7 (2022-10-12)
--------------------
* Fix launch substitutions for ign_args (`#309 <https://github.com/gazebosim/ros_gz/issues/309>`_)
  * Fix launch substitutions for ign_args
* Merge pull request `#275 <https://github.com/gazebosim/ros_gz/issues/275>`_ (Galactic to Humble)
  Galactic to Humble
* Merge branch 'ros2' into ports/galactic_to_ros2
* Contributors: Michael Carroll

0.244.6 (2022-09-14)
--------------------

0.244.5 (2022-09-12)
--------------------
* Fix missing msgs include and packages.xml deps (`#292 <https://github.com/gazebosim/ros_gz/issues/292>`_)
  * Fix missing msgs include and packages.xml deps
  * Add additional conditions to support gz sim invocation
  * Fix cpplint
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
* Fix linter tests (`#251 <https://github.com/gazebosim/ros_gz/issues/251>`_)
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Contributors: Daisuke Nishimatsu, Louise Poubel

0.244.2 (2022-04-25)
--------------------
* Support bridging services (`#211 <https://github.com/gazebosim/ros_gz/issues/211>`_)
* Add std_msgs as dependency of ros_gz_sim (`#242 <https://github.com/gazebosim/ros_gz/issues/242>`_)
* Fixed ros_gz_sim launch file install directory (`#229 <https://github.com/gazebosim/ros_gz/issues/229>`_) (`#230 <https://github.com/gazebosim/ros_gz/issues/230>`_)
* Added ign_version launch argument to set ignition gazebo version (`#226 <https://github.com/gazebosim/ros_gz/issues/226>`_)
* Bring ros2 branch up-to-date with Rolling (`#213 <https://github.com/gazebosim/ros_gz/issues/213>`_)
* create.cpp usage message fixed for ros2 branch (`#207 <https://github.com/gazebosim/ros_gz/issues/207>`_)
* Separate galactic branch from ros2 branch (`#201 <https://github.com/gazebosim/ros_gz/issues/201>`_)
* 🏁 Dome EOL (`#198 <https://github.com/gazebosim/ros_gz/issues/198>`_)
* Contributors: Alejandro Hernández Cordero, Aryaman Shardul, Ivan Santiago Paunovic, Kenji Brameld, Louise Poubel, Michael Carroll, ahcorde

0.244.1 (2022-01-04)
--------------------

0.244.0 (2021-12-30)
--------------------
* Default to Fortress for Rolling (future Humble) (`#195 <https://github.com/gazebosim/ros_gz/issues/195>`_)
* [ros2] 🏁 Dome EOL (`#199 <https://github.com/gazebosim/ros_gz/issues/199>`_)
* Contributors: Guillaume Doisy, Louise Poubel

0.233.2 (2021-07-20)
--------------------
* [ros2] Update version docs, add Galactic and Fortress (`#164 <https://github.com/gazebosim/ros_gz/issues/164>`_)
* Contributors: Louise Poubel

0.233.1 (2021-04-16)
--------------------
* Default to Edifice for Rolling (`#150 <https://github.com/gazebosim/ros_gz/issues/150>`_)
* Edifice support (`#140 <https://github.com/gazebosim/ros_gz/issues/140>`_)
  Co-authored-by: Alejandro Hernández <ahcorde@gmail.com>
* Add topic flag to create robot  (`#128 <https://github.com/gazebosim/ros_gz/issues/128>`_)
  Now it is possible to run ros_gz_sim create specifying a topic as
  source of the robot description
  Add a launch file starting a ignition gazebo world and spawn a sphere in it.
  Additionally a rviz2 interface is loaded to show that also Rviz can load
  the robot description
  The newly created demo introduce a dependency on the robot_state_publisher package
* Add default value for plugin path in launch script (`#125 <https://github.com/gazebosim/ros_gz/issues/125>`_)
* Fix overwriting of plugin path in launch script (`#122 <https://github.com/gazebosim/ros_gz/issues/122>`_)
  - GZ_SIM_SYSTEM_PLUGIN_PATH was overwritten by LD_LIBRARY_PATH
  - Now it is instead extended by LD_LIBRARY_PATH
  - This allows use of gz_sim.launch.py with custom gazebo plugins
* Changed for loading xml from ROS param(`#119 <https://github.com/gazebosim/ros_gz/issues/119>`_) (`#120 <https://github.com/gazebosim/ros_gz/issues/120>`_)
* ros_gz_sim exec depend on gz-sim (`#110 <https://github.com/gazebosim/ros_gz/issues/110>`_)
* Update releases (`#108 <https://github.com/gazebosim/ros_gz/issues/108>`_)
* Add support for Dome (`#103 <https://github.com/gazebosim/ros_gz/issues/103>`_)
* Contributors: Andrej Orsula, Louise Poubel, Luca Della Vedova, Valerio Magnago, chama1176

0.221.1 (2020-08-19)
--------------------
* Add pkg-config as a buildtool dependency (`#102 <https://github.com/gazebosim/ros_gz/issues/102>`_)
* Contributors: Louise Poubel

0.221.0 (2020-07-23)
--------------------
* [ros2] Fixed CI - Added Foxy (`#89 <https://github.com/gazebosim/ros_gz/issues/89>`_)
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Added ros_gz_sim for ros2 (`#80 <https://github.com/gazebosim/ros_gz/issues/80>`_)
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
* Update Dashing docs (`#62 <https://github.com/gazebosim/ros_gz/issues/62>`_)
* Contributors: Alejandro Hernández Cordero, Louise Poubel, chapulina
