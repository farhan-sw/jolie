from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jolie_utility',
            executable='pwm_publisher_node',
            name='pwm_publisher_node',
            parameters=[{
                'scenario_file': 'scenario.yaml'
            }]
        )
    ])
