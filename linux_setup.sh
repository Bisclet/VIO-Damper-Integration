#! /bin/bash

if [[ $(/usr/bin/id -u) -ne 0 ]]; then
    echo "Not running as root"
    exit
fi

./BASALT_ROS2/scripts/install_deps.sh

## Install MAVROS and dependencies

apt install -y ros-$ROS_DISTRO-mavros*
source /opt/ros/$ROS_DISTRO/setup.bash
/opt/ros/$ROS_DISTRO/lib/mavros/install_geographiclib_datasets.sh 

## Enable odom and gps_rtk mavros plugins

sudo tee /opt/ros/humble/share/mavros/launch/px4_pluginlists.yaml > /dev/null <<'EOF'
/**:
  ros__parameters:
    plugin_denylist:
      # common

      # extras
      - image_pub
      - vibration
      - distance_sensor
      - rangefinder
      - wheel_odometry

    plugin_allowlist:
      - odom
      - gps_rtk
EOF
