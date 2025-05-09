import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    
    use_lidar_arg = DeclareLaunchArgument(
        name='use_lidar',
        default_value='true',
        description='Whether to use the LIDAR'
    )
    use_lidar = LaunchConfiguration("use_lidar")
    
    use_imu_arg = DeclareLaunchArgument(
        name='use_imu',
        default_value='true',
        description='Whether to use the IMU'
    )
    use_imu = LaunchConfiguration("use_imu")
    
    
    controller = IncludeLaunchDescription(
        launch_description_source=os.path.join(
            get_package_share_directory("sirius_controller"), 
            "launch", 
            "controller.launch.py"
        ),
        launch_arguments={
            "controller_type": "omniwheel"
        }.items()
    )
    
    hardware_interface = IncludeLaunchDescription(
        launch_description_source=os.path.join(
            get_package_share_directory("jolie_firmware"), 
            "launch", 
            "hardware_interface.launch.py"
        )
    )
    
    twist_mux = IncludeLaunchDescription(
        launch_description_source=os.path.join(
            get_package_share_directory("sirius_controller"), 
            "launch", 
            "twist_mux.launch.py"
        )
    )
    
    
    # LAUNCH HARDWARE
    
    lidar = IncludeLaunchDescription(
        launch_description_source=os.path.join(
            get_package_share_directory("jolie_firmware"), 
            "launch", 
            "lidarC1.launch.py"
        ),
        condition=IfCondition(LaunchConfiguration("use_lidar")
        ),
    )
    
    imu = IncludeLaunchDescription(
        launch_description_source=os.path.join(
            get_package_share_directory("wit_ros2_imu"), 
            "rviz_and_imu.launch.py"
        ),
        condition=IfCondition(LaunchConfiguration("use_imu")
        ),
    )
        
    return LaunchDescription([
        use_lidar_arg,
        use_imu_arg,
        hardware_interface,
        controller,
        twist_mux,
        lidar,
        imu
    ])