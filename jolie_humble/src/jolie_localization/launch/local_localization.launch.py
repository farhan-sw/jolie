from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():
    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False"
    )
    
    use_complementary_filter_arg = DeclareLaunchArgument(
        "use_complementary_filter",
        default_value="true",
        description="Whether to use the complementary filter"
    )
    
    use_complementary_filter = LaunchConfiguration("use_complementary_filter")
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
    
    complementary_filter = IncludeLaunchDescription(
        launch_description_source=os.path.join(
            get_package_share_directory("jolie_localization"), 
            "launch", 
            "complementary_filter.launch.py"
        ),
        condition=IfCondition(use_complementary_filter),
    )
    
    return LaunchDescription([
        use_python_arg,
        use_complementary_filter_arg,
        robot_localization,
        complementary_filter,
    ])