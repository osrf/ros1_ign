#!/bin/bash
set -ev

# Configuration.
export COLCON_WS=~/ws
export COLCON_WS_SRC=${COLCON_WS}/src
export DEBIAN_FRONTEND=noninteractive
export ROS_PYTHON_VERSION=3

apt update -qq
apt install -qq -y lsb-release wget curl build-essential

<<<<<<< HEAD
if [ "$GZ_VERSION" == "garden" ]; then
  echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list
  wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

  GZ_DEPS="libgz-sim7-dev"

  ROSDEP_ARGS="--skip-keys='sdformat-urdf'"
elif [ "$GZ_VERSION" == "harmonic" ]; then
  echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list
  wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

  GZ_DEPS="libgz-sim8-dev"

  ROSDEP_ARGS="--skip-keys='sdformat-urdf'"
fi

# Fortress comes through rosdep for Focal and Jammy
=======
echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list
wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
>>>>>>> 63b651a (Garden EOL (#662))

# Dependencies.
echo "deb http://packages.ros.org/ros2-testing/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-testing.list
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
apt-get update -qq
<<<<<<< HEAD
apt-get install -y $GZ_DEPS \
                       python3-colcon-common-extensions \
                       python3-rosdep
=======
apt-get install -y python3-colcon-common-extensions \
                   python3-rosdep
>>>>>>> 63b651a (Garden EOL (#662))

rosdep init
rosdep update
rosdep install --from-paths ./ -i -y -r --rosdistro $ROS_DISTRO $ROSDEP_ARGS

# Build.
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir -p $COLCON_WS_SRC
cp -r $GITHUB_WORKSPACE $COLCON_WS_SRC
cd $COLCON_WS
colcon build --event-handlers console_direct+

# Tests.
colcon test --event-handlers console_direct+
colcon test-result
