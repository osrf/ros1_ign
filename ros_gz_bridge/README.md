# Bridge communication between ROS and Gazebo

This package provides a network bridge which enables the exchange of messages
between ROS and Gazebo Transport.

The following message types can be bridged for topics:

<<<<<<< HEAD
| ROS type                                    | Gazebo type                                 |
|---------------------------------------------|:-------------------------------------------:|
| builtin_interfaces/msg/Time                 | ignition::msgs::Time                        |
| std_msgs/msg/Bool                           | ignition::msgs::Boolean                     |
| std_msgs/msg/ColorRGBA                      | ignition::msgs::Color                       |
| std_msgs/msg/Empty                          | ignition::msgs::Empty                       |
| std_msgs/msg/Float32                        | ignition::msgs::Float                       |
| std_msgs/msg/Float64                        | ignition::msgs::Double                      |
| std_msgs/msg/Header                         | ignition::msgs::Header                      |
| std_msgs/msg/Int32                          | ignition::msgs::Int32                       |
| std_msgs/msg/UInt32                         | ignition::msgs::UInt32                      |
| std_msgs/msg/String                         | ignition::msgs::StringMsg                   |
| geometry_msgs/msg/Wrench                    | ignition::msgs::Wrench                      |
| geometry_msgs/msg/WrenchStamped             | ignition::msgs::Wrench                      |
| geometry_msgs/msg/Quaternion                | ignition::msgs::Quaternion                  |
| geometry_msgs/msg/Vector3                   | ignition::msgs::Vector3d                    |
| geometry_msgs/msg/Point                     | ignition::msgs::Vector3d                    |
| geometry_msgs/msg/Pose                      | ignition::msgs::Pose                        |
| geometry_msgs/msg/PoseArray                 | ignition::msgs::Pose_V                      |
| geometry_msgs/msg/PoseWithCovariance        | ignition::msgs::PoseWithCovariance          |
| geometry_msgs/msg/PoseStamped               | ignition::msgs::Pose                        |
| geometry_msgs/msg/Transform                 | ignition::msgs::Pose                        |
| geometry_msgs/msg/TransformStamped          | ignition::msgs::Pose                        |
| geometry_msgs/msg/Twist                     | ignition::msgs::Twist                       |
| geometry_msgs/msg/TwistStamped              | ignition::msgs::Twist                       |
| geometry_msgs/msg/TwistWithCovariance       | ignition::msgs::TwistWithCovariance         |
| geometry_msgs/msg/TwistWithCovarianceStamped| ignition::msgs::TwistWithCovariance         |
| gps_msgs/GPSFix                             | ignition::msgs::NavSat                      |
| nav_msgs/msg/Odometry                       | ignition::msgs::Odometry                    |
| nav_msgs/msg/Odometry                       | ignition::msgs::OdometryWithCovariance      |
| rcl_interfaces/msg/ParameterValue           | ignition::msgs::Any                         |
| ros_gz_interfaces/msg/Altimeter             | ignition::msgs::Altimeter                   |
| ros_gz_interfaces/msg/Contact               | ignition::msgs::Contact                     |
| ros_gz_interfaces/msg/Contacts              | ignition::msgs::Contacts                    |
| ros_gz_interfaces/msg/Dataframe             | ignition::msgs::Dataframe                   |
| ros_gz_interfaces/msg/Entity                | ignition::msgs::Entity                      |
| ros_gz_interfaces/msg/EntityWrench          | ignition::msgs::EntityWrench                  |
| ros_gz_interfaces/msg/Float32Array          | ignition::msgs::Float_V                     |
| ros_gz_interfaces/msg/GuiCamera             | ignition::msgs::GUICamera                   |
| ros_gz_interfaces/msg/JointWrench           | ignition::msgs::JointWrench                 |
| ros_gz_interfaces/msg/Light                 | ignition::msgs::Light                       |
| ros_gz_interfaces/msg/SensorNoise           | ignition::msgs::SensorNoise                 |
| ros_gz_interfaces/msg/StringVec             | ignition::msgs::StringMsg_V                 |
| ros_gz_interfaces/msg/TrackVisual           | ignition::msgs::TrackVisual                 |
| ros_gz_interfaces/msg/VideoRecord           | ignition::msgs::VideoRecord                 |
| ros_gz_interfaces/msg/WorldControl          | ignition::msgs::WorldControl                |
| rosgraph_msgs/msg/Clock                     | ignition::msgs::Clock                       |
| sensor_msgs/msg/BatteryState                | ignition::msgs::BatteryState                |
| sensor_msgs/msg/CameraInfo                  | ignition::msgs::CameraInfo                  |
| sensor_msgs/msg/FluidPressure               | ignition::msgs::FluidPressure               |
| sensor_msgs/msg/Imu                         | ignition::msgs::IMU                         |
| sensor_msgs/msg/Image                       | ignition::msgs::Image                       |
| sensor_msgs/msg/JointState                  | ignition::msgs::Model                       |
| sensor_msgs/msg/Joy                         | ignition::msgs::Joy                         |
| sensor_msgs/msg/LaserScan                   | ignition::msgs::LaserScan                   |
| sensor_msgs/msg/MagneticField               | ignition::msgs::Magnetometer                |
| sensor_msgs/msg/NavSatFix                   | ignition::msgs::NavSat                      |
| sensor_msgs/msg/PointCloud2                 | ignition::msgs::PointCloudPacked            |
| tf2_msgs/msg/TFMessage                      | ignition::msgs::Pose_V                      |
| trajectory_msgs/msg/JointTrajectory         | ignition::msgs::JointTrajectory             |
| vision_msgs/msg/Detection3D                 | ignition::msgs::AnnotatedOriented3DBox      |
| vision_msgs/msg/Detection3DArray            | ignition::msgs::AnnotatedOriented3DBox_V    |
=======
| ROS type                                       | Gazebo Transport Type               |
| ---------------------------------------------- | :------------------------------:    |
| actuator_msgs/msg/Actuators                    | gz.msgs.Actuators                   |
| builtin_interfaces/msg/Time                    | gz.msgs.Time                        |
| geometry_msgs/msg/Point                        | gz.msgs.Vector3d                    |
| geometry_msgs/msg/Pose                         | gz.msgs.Pose                        |
| geometry_msgs/msg/PoseArray                    | gz.msgs.Pose_V                      |
| geometry_msgs/msg/PoseStamped                  | gz.msgs.Pose                        |
| geometry_msgs/msg/PoseWithCovariance           | gz.msgs.PoseWithCovariance          |
| geometry_msgs/msg/PoseWithCovarianceStamped    | gz.msgs.PoseWithCovariance          |
| geometry_msgs/msg/Quaternion                   | gz.msgs.Quaternion                  |
| geometry_msgs/msg/Transform                    | gz.msgs.Pose                        |
| geometry_msgs/msg/TransformStamped             | gz.msgs.Pose                        |
| geometry_msgs/msg/Twist                        | gz.msgs.Twist                       |
| geometry_msgs/msg/TwistStamped                 | gz.msgs.Twist                       |
| geometry_msgs/msg/TwistWithCovariance          | gz.msgs.TwistWithCovariance         |
| geometry_msgs/msg/TwistWithCovarianceStamped   | gz.msgs.TwistWithCovariance         |
| geometry_msgs/msg/Vector3                      | gz.msgs.Vector3d                    |
| geometry_msgs/msg/Wrench                       | gz.msgs.Wrench                      |
| geometry_msgs/msg/WrenchStamped                | gz.msgs.Wrench                      |
| gps_msgs/msg/GPSFix                            | gz.msgs.NavSat                      |
| nav_msgs/msg/Odometry                          | gz.msgs.Odometry                    |
| nav_msgs/msg/Odometry                          | gz.msgs.OdometryWithCovariance      |
| rcl_interfaces/msg/ParameterValue              | gz.msgs.Any                         |
| ros_gz_interfaces/msg/Altimeter                | gz.msgs.Altimeter                   |
| ros_gz_interfaces/msg/Contact                  | gz.msgs.Contact                     |
| ros_gz_interfaces/msg/Contacts                 | gz.msgs.Contacts                    |
| ros_gz_interfaces/msg/Dataframe                | gz.msgs.Dataframe                   |
| ros_gz_interfaces/msg/Entity                   | gz.msgs.Entity                      |
| ros_gz_interfaces/msg/EntityWrench             | gz.msgs.EntityWrench                |
| ros_gz_interfaces/msg/Float32Array             | gz.msgs.Float_V                     |
| ros_gz_interfaces/msg/GuiCamera                | gz.msgs.GUICamera                   |
| ros_gz_interfaces/msg/JointWrench              | gz.msgs.JointWrench                 |
| ros_gz_interfaces/msg/Light                    | gz.msgs.Light                       |
| ros_gz_interfaces/msg/ParamVec                 | gz.msgs.Param                       |
| ros_gz_interfaces/msg/ParamVec                 | gz.msgs.Param_V                     |
| ros_gz_interfaces/msg/SensorNoise              | gz.msgs.SensorNoise                 |
| ros_gz_interfaces/msg/StringVec                | gz.msgs.StringMsg_V                 |
| ros_gz_interfaces/msg/TrackVisual              | gz.msgs.TrackVisual                 |
| ros_gz_interfaces/msg/VideoRecord              | gz.msgs.VideoRecord                 |
| rosgraph_msgs/msg/Clock                        | gz.msgs.Clock                       |
| sensor_msgs/msg/BatteryState                   | gz.msgs.BatteryState                |
| sensor_msgs/msg/CameraInfo                     | gz.msgs.CameraInfo                  |
| sensor_msgs/msg/FluidPressure                  | gz.msgs.FluidPressure               |
| sensor_msgs/msg/Image                          | gz.msgs.Image                       |
| sensor_msgs/msg/Imu                            | gz.msgs.IMU                         |
| sensor_msgs/msg/JointState                     | gz.msgs.Model                       |
| sensor_msgs/msg/Joy                            | gz.msgs.Joy                         |
| sensor_msgs/msg/LaserScan                      | gz.msgs.LaserScan                   |
| sensor_msgs/msg/MagneticField                  | gz.msgs.Magnetometer                |
| sensor_msgs/msg/NavSatFix                      | gz.msgs.NavSat                      |
| sensor_msgs/msg/PointCloud2                    | gz.msgs.PointCloudPacked            |
| std_msgs/msg/Bool                              | gz.msgs.Boolean                     |
| std_msgs/msg/ColorRGBA                         | gz.msgs.Color                       |
| std_msgs/msg/Empty                             | gz.msgs.Empty                       |
| std_msgs/msg/Float32                           | gz.msgs.Float                       |
| std_msgs/msg/Float64                           | gz.msgs.Double                      |
| std_msgs/msg/Header                            | gz.msgs.Header                      |
| std_msgs/msg/Int32                             | gz.msgs.Int32                       |
| std_msgs/msg/String                            | gz.msgs.StringMsg                   |
| std_msgs/msg/UInt32                            | gz.msgs.UInt32                      |
| tf2_msgs/msg/TFMessage                         | gz.msgs.Pose_V                      |
| trajectory_msgs/msg/JointTrajectory            | gz.msgs.JointTrajectory             |
| vision_msgs/msg/Detection2D                    | gz.msgs.AnnotatedAxisAligned2DBox   |
| vision_msgs/msg/Detection2DArray               | gz.msgs.AnnotatedAxisAligned2DBox_V |
| vision_msgs/msg/Detection3D                    | gz::msgs::AnnotatedOriented3DBox    |
| vision_msgs/msg/Detection3DArray               | gz::msgs::AnnotatedOriented3DBox_V  |
>>>>>>> 63b651a (Garden EOL (#662))

And the following for services:

| ROS type                             | Gazebo request             | Gazebo response       |
|--------------------------------------|:--------------------------:| --------------------- |
<<<<<<< HEAD
| ros_gz_interfaces/srv/ControlWorld   | ignition.msgs.WorldControl | ignition.msgs.Boolean |

Run `ros2 run ros_gz_bridge parameter_bridge -h` for instructions.

=======
| ros_gz_interfaces/srv/ControlWorld   | gz.msgs.WorldControl       | gz.msgs.Boolean       |

Run `ros2 run ros_gz_bridge parameter_bridge -h` for instructions.

**NOTE**: If during startup, gazebo detects that there is another publisher on `/clock`, it will only create the fully qualified `/world/<worldname>/clock topic`.
Gazebo would be the only `/clock` publisher, the sole source of clock information.

You should create an unidirectional `/clock` bridge:

```bash
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

An alternative set-up can be using the bridge with the `override_timestamps_with_wall_time` ros parameter set to `true` (default=`false`). In this set-up,
all header timestamps of the outgoing messages will be stamped with the wall time. This can be useful when the simulator has to communicate with an external system that requires wall times.

>>>>>>> 63b651a (Garden EOL (#662))
## Example 1a: Gazebo Transport talker and ROS 2 listener

Start the parameter bridge which will watch the specified topics.

```
# Shell A:
. ~/bridge_ws/install/setup.bash
<<<<<<< HEAD
ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/msg/String@ignition.msgs.StringMsg
=======
ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/msg/String@gz.msgs.StringMsg
>>>>>>> 63b651a (Garden EOL (#662))
```

Now we start the ROS listener.

```
# Shell B:
<<<<<<< HEAD
. /opt/ros/humble/setup.bash
=======
. /opt/ros/rolling/setup.bash
>>>>>>> 63b651a (Garden EOL (#662))
ros2 topic echo /chatter
```

Now we start the Gazebo Transport talker.

```
# Shell C:
<<<<<<< HEAD
ign topic -t /chatter -m ignition.msgs.StringMsg -p 'data:"Hello"'
=======
gz topic -t /chatter -m gz.msgs.StringMsg -p 'data:"Hello"'
>>>>>>> 63b651a (Garden EOL (#662))
```

## Example 1b: ROS 2 talker and Gazebo Transport listener

Start the parameter bridge which will watch the specified topics.

```
# Shell A:
. ~/bridge_ws/install/setup.bash
<<<<<<< HEAD
ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/msg/String@ignition.msgs.StringMsg
=======
ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/msg/String@gz.msgs.StringMsg
>>>>>>> 63b651a (Garden EOL (#662))
```

Now we start the Gazebo Transport listener.

```
# Shell B:
<<<<<<< HEAD
ign topic -e -t /chatter
=======
gz topic -e -t /chatter
>>>>>>> 63b651a (Garden EOL (#662))
```

Now we start the ROS talker.

```
# Shell C:
<<<<<<< HEAD
. /opt/ros/humble/setup.bash
=======
. /opt/ros/rolling/setup.bash
>>>>>>> 63b651a (Garden EOL (#662))
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hi'" --once
```

## Example 2: Run the bridge and exchange images

In this example, we're going to generate Gazebo Transport images using
Gazebo Sim, that will be converted into ROS images, and visualized with
`rqt_image_viewer`.

First we start Gazebo Sim (don't forget to hit play, or Gazebo Sim won't generate any images).

```
# Shell A:
<<<<<<< HEAD
ign gazebo sensors_demo.sdf
=======
gz sim sensors_demo.sdf
>>>>>>> 63b651a (Garden EOL (#662))
```

Let's see the topic where camera images are published.

```
# Shell B:
<<<<<<< HEAD
ign topic -l | grep image
=======
gz topic -l | grep image
>>>>>>> 63b651a (Garden EOL (#662))
/rgbd_camera/depth_image
/rgbd_camera/image
```

Then we start the parameter bridge with the previous topic.

```
# Shell B:
. ~/bridge_ws/install/setup.bash
<<<<<<< HEAD
ros2 run ros_gz_bridge parameter_bridge /rgbd_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image
=======
ros2 run ros_gz_bridge parameter_bridge /rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image
>>>>>>> 63b651a (Garden EOL (#662))
```

Now we start the ROS GUI:

```
# Shell C:
<<<<<<< HEAD
. /opt/ros/humble/setup.bash
=======
. /opt/ros/rolling/setup.bash
>>>>>>> 63b651a (Garden EOL (#662))
ros2 run rqt_image_view rqt_image_view /rgbd_camera/image
```

You should see the current images in `rqt_image_view` which are coming from
Gazebo (published as Gazebo Msgs over Gazebo Transport).

The screenshot shows all the shell windows and their expected content
(it was taken using ROS 2 Galactic and Gazebo Fortress):

![Gazebo Transport images and ROS rqt](images/bridge_image_exchange.png)

## Example 3: Static bridge

In this example, we're going to run an executable that starts a bidirectional
bridge for a specific topic and message type. We'll use the `static_bridge`
executable that is installed with the bridge.

The example's code can be found under `ros_gz_bridge/src/static_bridge.cpp`.
In the code, it's possible to see how the bridge is hardcoded to bridge string
messages published on the `/chatter` topic.

Let's give it a try, starting with Gazebo -> ROS 2.

On terminal A, start the bridge:

`ros2 run ros_gz_bridge static_bridge`

On terminal B, we start a ROS 2 listener:

`ros2 topic echo /chatter std_msgs/msg/String`

And terminal C, publish an Gazebo message:

<<<<<<< HEAD
`ign topic -t /chatter -m ignition.msgs.StringMsg -p 'data:"Hello"'`
=======
`gz topic -t /chatter -m gz.msgs.StringMsg -p 'data:"Hello"'`
>>>>>>> 63b651a (Garden EOL (#662))

At this point, you should see the ROS 2 listener echoing the message.

Now let's try the other way around, ROS 2 -> Gazebo.

<<<<<<< HEAD
On terminal D, start an Igntion listener:

`ign topic -e -t /chatter`
=======
On terminal D, start an Gazebo listener:

`gz topic -e -t /chatter`
>>>>>>> 63b651a (Garden EOL (#662))

And on terminal E, publish a ROS 2 message:

`ros2 topic pub /chatter std_msgs/msg/String 'data: "Hello"' -1`

You should see the Gazebo listener echoing the message.

## Example 4: Service bridge

It's possible to make ROS service requests into Gazebo. Let's try unpausing the simulation.

On terminal A, start the service bridge:

`ros2 run ros_gz_bridge parameter_bridge /world/shapes/control@ros_gz_interfaces/srv/ControlWorld`

On terminal B, start Gazebo, it will be paused by default:

<<<<<<< HEAD
`ign gazebo shapes.sdf`
=======
`gz sim shapes.sdf`
>>>>>>> 63b651a (Garden EOL (#662))

On terminal C, make a ROS request to unpause simulation:

```
ros2 service call /world/<world_name>/control ros_gz_interfaces/srv/ControlWorld "{world_control: {pause: false}}"
```

## Example 5: Configuring the Bridge via YAML

When configuring many topics, it is easier to use a file-based configuration in a markup
language. In this case, the `ros_gz` bridge supports using a YAML file to configure the
various parameters.

The configuration file must be a YAML array of maps.
An example configuration for 5 bridges is below, showing the various ways that a
bridge may be specified:

```yaml
 # Set just topic name, applies to both
- topic_name: "chatter"
  ros_type_name: "std_msgs/msg/String"
<<<<<<< HEAD
  gz_type_name: "ignition.msgs.StringMsg"
=======
  gz_type_name: "gz.msgs.StringMsg"
>>>>>>> 63b651a (Garden EOL (#662))

# Set just ROS topic name, applies to both
- ros_topic_name: "chatter_ros"
  ros_type_name: "std_msgs/msg/String"
<<<<<<< HEAD
  gz_type_name: "ignition.msgs.StringMsg"

# Set just GZ topic name, applies to both
- gz_topic_name: "chatter_ign"
  ros_type_name: "std_msgs/msg/String"
  gz_type_name: "ignition.msgs.StringMsg"

# Set each topic name explicitly
- ros_topic_name: "chatter_both_ros"
  gz_topic_name: "chatter_both_ign"
  ros_type_name: "std_msgs/msg/String"
  gz_type_name: "ignition.msgs.StringMsg"

# Full set of configurations
- ros_topic_name: "ros_chatter"
  gz_topic_name: "ign_chatter"
  ros_type_name: "std_msgs/msg/String"
  gz_type_name: "ignition.msgs.StringMsg"
=======
  gz_type_name: "gz.msgs.StringMsg"

# Set just GZ topic name, applies to both
- gz_topic_name: "chatter_gz"
  ros_type_name: "std_msgs/msg/String"
  gz_type_name: "gz.msgs.StringMsg"

# Set each topic name explicitly
- ros_topic_name: "chatter_both_ros"
  gz_topic_name: "chatter_both_gz"
  ros_type_name: "std_msgs/msg/String"
  gz_type_name: "gz.msgs.StringMsg"

# Full set of configurations
- ros_topic_name: "ros_chatter"
  gz_topic_name: "gz_chatter"
  ros_type_name: "std_msgs/msg/String"
  gz_type_name: "gz.msgs.StringMsg"
>>>>>>> 63b651a (Garden EOL (#662))
  subscriber_queue: 5       # Default 10
  publisher_queue: 6        # Default 10
  lazy: true                # Default "false"
  direction: BIDIRECTIONAL  # Default "BIDIRECTIONAL" - Bridge both directions
<<<<<<< HEAD
                            # "GZ_TO_ROS" - Bridge Ignition topic to ROS
                            # "ROS_TO_GZ" - Bridge ROS topic to Ignition
=======
                            # "GZ_TO_ROS" - Bridge Gz topic to ROS
                            # "ROS_TO_GZ" - Bridge ROS topic to Gz
>>>>>>> 63b651a (Garden EOL (#662))
```

To run the bridge node with the above configuration:
```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$WORKSPACE/ros_gz/ros_gz_bridge/test/config/full.yaml
```

## Example 6: Using ROS namespace with the Bridge

When spawning multiple robots inside the same ROS environment, it is convenient to use namespaces to avoid overlapping topic names.
There are three main types of namespaces: relative, global (`/`) and private (`~/`). For more information, refer to ROS documentation.
Namespaces are applied to Gazebo topic both when specified as `topic_name` as well as `gz_topic_name`.

By default, the Bridge will not apply ROS namespace on the Gazebo topics. To enable this feature, use parameter `expand_gz_topic_names`.
Let's test our topic with namespace:

```bash
# Shell A:
. ~/bridge_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge chatter@std_msgs/msg/String@ignition.msgs.StringMsg \
  --ros-args -p expand_gz_topic_names:=true -r __ns:=/demo
```

Now we start the Gazebo Transport listener.

```bash
# Shell B:
<<<<<<< HEAD
ign topic -e -t /demo/chatter
=======
gz topic -e -t /demo/chatter
>>>>>>> 63b651a (Garden EOL (#662))
```

Now we start the ROS talker.

```bash
# Shell C:
<<<<<<< HEAD
. /opt/ros/humble/setup.bash
=======
. /opt/ros/rolling/setup.bash
>>>>>>> 63b651a (Garden EOL (#662))
ros2 topic pub /demo/chatter std_msgs/msg/String "data: 'Hi from inside of a namespace'" --once
```

By changing `chatter` to `/chatter` or `~/chatter` you can obtain different results.

## API

ROS 2 Parameters:

 * `subscription_heartbeat` - Period at which the node checks for new subscribers for lazy bridges.
 * `config_file` - YAML file to be loaded as the bridge configuration
 * `expand_gz_topic_names` - Enable or disable ROS namespace applied on GZ topics.
