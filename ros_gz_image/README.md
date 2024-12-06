# Image utilities for using ROS and Gazebo Transport

This package provides a unidirectional bridge for images from Gazebo to ROS.
<<<<<<< HEAD
The bridge subscribes to Gazebo image messages (`ignition::msgs::Image`)
=======
The bridge subscribes to Gazebo image messages (`gz::msgs::Image`)
>>>>>>> 63b651a (Garden EOL (#662))
and republishes them to ROS using [image_transport](http://wiki.ros.org/image_transport).

For compressed images, install
[compressed_image_transport](http://wiki.ros.org/compressed_image_transport)
and the bridge will publish `/compressed` images. The same goes for other
`image_transport` plugins.

<<<<<<< HEAD
=======
To run the bridge from the command line:

```shell
ros2 run ros_gz_image image_bridge /topic1 /topic2
```

You can also modify the [Quality of Service (QoS) policy](https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html#qos-policies) used to publish images using an additional `qos` ROS parameter. For example:

```shell
ros2 run ros_gz_image image_bridge /topic1 /topic2 --ros-args qos:=sensor_data
```
>>>>>>> 63b651a (Garden EOL (#662))
