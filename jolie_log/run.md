# Run ESP
docker run -it --rm --net=host --device=/dev/ttyACM0 microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -b 921500

# Run ROS2 Nodes, Navigation, Localization
ros2 launch jolie_bringup real_robot.launch.py

ros2 launch jolie_localization global_localization.launch.py map_name:=9308

ros2 launch jolie_localization local_localization.launch.py

ros2 launch jolie_navigation navigation.launch.py

# Run SLAM
ros2 launch jolie_bringup real_robot.launch.py

ros2 launch jolie_mapping slam.launch.py