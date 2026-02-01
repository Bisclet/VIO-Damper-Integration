# VIO-Damper-Integration

This repository contains the integration of the [Basalt VIO pipeline](https://github.com/VladyslavUsenko/basalt), it includes a ROS2 wrapper of the original pipeline, metric tools and demo launch files.
## Prerequisites

This package assumes you installed the [ZED SDK](https://www.stereolabs.com/docs/development/zed-sdk/linux) and built the [ZED ROS2 NODE](https://github.com/stereolabs/zed-ros2-wrapper) inside your workspace. This package uses [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html), there is currently no official support for other distros.


## Building the package

```bash
# Clone this package into your workspace
cd <your_ws>/src
git clone --recursive https://github.com/Bisclet/VIO-Damper-Integration.git
cd ..

# Run the linux setup script, this installs necessary dependencies for Basalt
./src/VIO-Damper-Integration/linux_setup.sh

# Build packages, it is highly recommended to build using --symlink-install
colcon build --symlink-install

# Source your workspace
source install/setup.bash # If using bash
source install/setup.zsh  # If using zsh
```

## Running the package

The package has several launch files:

*1)* Full Demo: Starts the ZED wrapper, Basalt, metric tools, rosbag recording and ground truth estimation.
```bash
ros2 launch cocoro_evaluation full_demo.launch.py
```

*2)* Demo without rosbag recording and ground truth estimation.
```bash
ros2 launch cocoro_evaluation demo.launch.py
```

*3)* Standalone VIO-Pipeline
```bash
ros2 launch basalt basalt_vio.launch.py \
        left_image_topic:=/zed/zed_node/left/color/rect/image \
        right_image_topic:=/zed/zed_node/right/color/rect/image \
        imu_topic:=/zed/zed_node/imu/data_raw \
        odometry_topic:=/basalt/odom \
        cam_calib:=$(realpath ./install/basalt/share/basalt/config/zed2i_pinhole_calib.json) \
        config_path:=$(realpath ./install/basalt/share/basalt/config/zed2i_config.json)
```

If you wish to change the topic names of the demo launch files you can do it as follows:

```bash
cd <your_ws>
nano ./src/VIO-Damper-Integration/launch/full_demo.launch.py # Full demo launch file
nano ./src/VIO-Damper-Integration/launch/demo.launch.py      # Half demo launch file
```

## Remote Debugging

While running on the damper you won't have visual access to the onboard computer, that's why you can use the provided DDS setup to transfer topics over network. This script is based on the [Zenoh-ROS2-DDS](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds), you must be in the same network as the companion computer, it can be EDUROAM, a hotspot or a VPN (e.g. Tailscale). This DDS config does not intefere with other DDS configurations you might have running. This part assumes you have [Docker](https://docs.docker.com/engine/install/ubuntu/) installed.

You need to do the same setup on your PC running ROS2-Humble and your companion computer:

*1)* Modify the zenoh configuration file:

```bash
cd <your_ws>/src/VIO-Damper-Integration/DDS
nano assets/zenoh-config.json5
```

by replacing the <peer_ip> placeholder with the machines peer ip.

```json5
...
connect: {
    endpoints: [
      "tcp/<peer_ip>:7447"
    ]
  }
...
```

*2)* Start the DDSs docker-compose file:

```bash
cd <your_ws>/src/VIO-Damper-Integration/DDS
docker compose up
```

You should be able to see the shared topics in both machines by running 

```bash
ros2 topic list
```

If you don't see the shared topics make sure you are in the same **ROS_DOMAIN_ID**, the DDS startup does not set a specific DOMAIN_ID, if you wish to do so modify the docker-compose.yaml by uncommenting **ROS_DOMAIN_ID=<your_ID>**, setting your desired ID and restarting the docker container.