from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[os.path.join(get_package_share_directory('sirius_controller'), 'config', 'twist_mux.yaml'),
                    ],
        remappings=[('/cmd_vel_out', '/twist_mux/cmd_vel')],
    )
    
    return LaunchDescription([
        twist_mux,
    ])
    