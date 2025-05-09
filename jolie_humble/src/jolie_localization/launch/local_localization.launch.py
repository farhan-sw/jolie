from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False"
    )
    use_python = LaunchConfiguration("use_python")

    # static_transform_publisher = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=["--x", "0", "--y", "0", "--z", "0.103", "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
    #               "--frame-id", "odom", "--child-frame-id", "base_link"]
    # )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("jolie_localization"), "config", "ekf.yaml")]
    )
    
    return LaunchDescription([
        use_python_arg,
        robot_localization,
    ])