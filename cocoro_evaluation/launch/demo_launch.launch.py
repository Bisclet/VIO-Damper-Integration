from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # --- ZED camera launch ---
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("zed_wrapper"),
                "launch",
                "zed_camera.launch.py"
            ])
        ),
        launch_arguments={
            "camera_model": "zed2i"
        }.items()
    )

    # --- Basalt VIO launch ---
    basalt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("basalt"),
                "launch",
                "basalt_vio.launch.py"
            ])
        ),
        launch_arguments={
            "left_image_topic": "/zed/zed_node/left/color/rect/image",
            "right_image_topic": "/zed/zed_node/right/color/rect/image",
            "imu_topic": "/zed/zed_node/imu/data_raw",
            "odometry_topic": "/basalt/odom",
            "cam_calib": PathJoinSubstitution([
                FindPackageShare("basalt"),
                "config",
                "zed2i_pinhole_calib.json"
            ]),
            "config_path": PathJoinSubstitution([
                FindPackageShare("basalt"),
                "config",
                "zed2i_config.json"
            ]),
        }.items()
    )

    # --- Metrics node ---
    metrics_node = Node(
        package="cocoro_evaluation",
        executable="measure_metrics",
        output="screen"
    )

    # --- Rosbag recording ---
    '''
    rosbag_record = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record",
            "/zed/zed_node/left/color/rect/image",
            "/basalt/odom"
        ],
        output="screen"
    )
    '''

    return LaunchDescription([
        zed_launch,
        basalt_launch,
        metrics_node,
        #rosbag_record
    ])
