# ROS + Gazebo Sim

This package contains things that make it convenient to integrate ROS with Gazebo, such as:

 - Launch files
 - ROS-enabled executables

### Run Gazebo Sim

There's a convenient launch file, try for example:

```bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="shapes.sdf"
```

### Spawn entities

The `create` executable can be used to spawn SDF or URDF entities from:

 - A file on disk or from Gazebo Fuel
 - A ROS parameter

For example, start Gazebo Sim:

```
ros2 launch ros_gz_sim gz_sim.launch.py
```

then spawn a model:

```
<<<<<<< HEAD
ros2 run ros_gz_sim create -world default -file 'https://fuel.ignitionrobotics.org/1.0/openrobotics/models/Gazebo'
=======
ros2 run ros_gz_sim create -world default -file 'https://fuel.gazebosim.org/1.0/openrobotics/models/Gazebo'
>>>>>>> 63b651a (Garden EOL (#662))
```

See more options with:

```
ros2 run ros_gz_sim create --helpshort
```

### Using `<gazebo_ros>` to export model paths in `package.xml`

The `<gazebo_ros>` tag inside the `<export>` tag of a `package.xml` file can be
used to add paths to `GZ_SIM_RESOURCE_PATH` and `GZ_SIM_SYSTEM_PLUGIN_PATH`,
which are environment variables used to configure Gazebo search paths for
resources (e.g. SDFormat files, meshes, etc) and plugins respectively.

The values in the attributes `gazebo_model_path` and `gazebo_media_path` are
appended to `GZ_SIM_RESOURCE_PATH`. The value of `plugin_path` is appended to
`GZ_SIM_SYSTEM_PLUGIN_PATH`. See the
[Finding resources](https://gazebosim.org/api/sim/8/resources.html) tutorial to
learn more about these environment variables.

The keyword `${prefix}` can be used when setting these values and it will be
expanded to the package's share path (i.e., the value of
`ros2 pkg prefix --share <package name>`)

```xml
<export>
   <gazebo_ros gazebo_model_path="${prefix}/models"/>
   <gazebo_ros gazebo_media_path="${prefix}/media"/>
   <gazebo_ros plugin_path="${prefix}/plugins"/>
</export>

```

Thus the required directory needs to be installed from `CMakeLists.txt`

```cmake
install(DIRECTORY models
    DESTINATION share/${PROJECT_NAME})
```

In order to reference the models in a ROS package unambiguously, it is
recommended to set the value of `gazebo_model_path` to be the parent
of the `prefix`.

```xml
<export>
   <gazebo_ros gazebo_model_path="${prefix}/../"/>
</export>

```

Consider an example where we have a ROS package called `my_awesome_pkg`
and it contains an SDFormat model cool `cool_robot`:

```bash
my_awesome_pkg
├── models
│   └── cool_robot
│       ├── model.config
│       └── model.sdf
└── package.xml
```

With `gazebo_model_path="${prefix}/../` set up, we can
reference the `cool_robot` model in a world file using the package name
in the `uri`:

```xml
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>package://my_awesome_pkg/models/cool_robot</uri>
    </include>
  </world>
</sdf>
```

However, if we set `gazebo_model_path=${prefix}/models`, we would
need to reference `cool_robot` as `package://cool_robot`, which
might have a name conflict with other models in the system.
