import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    
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
    
    lidar = IncludeLaunchDescription(
        launch_description_source=os.path.join(
            get_package_share_directory("jolie_firmware"), 
            "launch", 
            "lidarC1.launch.py"
        )
    )
        
    return LaunchDescription([
        hardware_interface,
        controller,
        twist_mux,
        lidar
    ])