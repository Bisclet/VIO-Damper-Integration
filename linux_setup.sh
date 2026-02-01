#! /bin/bash

sudo ./BASALT_ROS2/scripts/install_deps.sh

#- Install MAVROS and dependencies

sudo apt install -y ros-$ROS_DISTRO-mavros*
source /opt/ros/$ROS_DISTRO/setup.bash
sudo /opt/ros/$ROS_DISTRO/lib/mavros/install_geographiclib_datasets.sh 

#- Enable odom and gps_rtk mavros plugins

sudo tee /opt/ros/humble/share/mavros/launch/px4_pluginlists.yaml > /dev/null <<'EOF'
/**:
  ros__parameters:
    plugin_denylist:
      - actuator_control
      - adsb
      - altitude
      - cam_imu_sync
      - camera
      - cellular_status
      - command
      - companion_process_status
      - common
      - debug_value
      - distance_sensor
      - esc_status
      - esc_telemetry
      - extras
      - fake_gps
      - geofence
      - global_position
      - gps_input
      - gps_status
      - guided_target
      - hil
      - home_position
      - image_pub
      - imu
      - landing_target
      - local_position
      - log_transfer
      - mag_calibration_status
      - manual_control
      - mocap_pose_estimate
      - mount_control
      - nav_controller_output
      - obstacle
      - obstacle_distance
      - optical_flow
      - onboard_computer_status
      - param
      - play_tune
      - px4flow
      - rallypoint
      - rangefinder
      - rc_io
      - setpoint_accel
      - setpoint_attitude
      - setpoint_position
      - setpoint_raw
      - setpoint_trajectory
      - setpoint_velocity
      - sys_status
      - sys_time
      - tdr_radio
      - terrain
      - trajectory
      - tunnel
      - vfr_hud
      - vibration
      - vision_pose
      - vision_speed
      - vision_speed_estimate
      - waypoint
      - wheel_odometry
      - gimbal_control
      - obstacle_distance_3d
      - open_drone_id
      - sim_state
      - uas1
      - wind_estimation

    plugin_allowlist:
      - odom
      - gps_rtk
      - local_position
EOF
