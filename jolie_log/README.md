# LOG FILE

This directory contains the log develoment files for the Jolie Robot. Contains Date and command logs.

## Date Log
This file contains the date logs for each development progress. It contains the date and the progress made on that date.

### [2025-02-17]
- **Progress**: 
    - Created basic template for microros and setup docker for microros

    ```bash
    docker pull microros/micro-ros-agent:humble

    docker run -it --rm --net=host --device=/dev/ttyUSB0 microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0

    docker run -it --rm --net=host --device=/dev/ttyACM0 microros/micro-ros-agent:humble serial --dev /dev/ttyACM0
    ```
- **Issues**: 
    - The device is not general, if the device port changes, the docker command will not work. Need to find a way to make it general. e.g. /dev/ttyUSB0 -> esp32_base

- **Next Steps**: 
    - Make an OOP for the main C file, and integrate with the current base code
    - Make a folder contain libraries


### [2025-02-20]
- **Progress**: 
    - Import URDF model based on KRAI Description
    ```bash
    sudo apt-get install ros-humble-urdf-tutorial
    
    ros2 launch urdf_tutorial display.launch.py model:=/home/farhan-sw/Documents/GitHub/jolie/jolie_humble/src/jolie_description/urdf/nebula_4wd.urdf.xacro
    ```

- **References**:
    - https://github.com/hippo5329/micro_ros_arduino_examples_platformio
    - https://github.com/hippo5329/micro_ros_arduino_examples_platformio/wiki

### [2025-05-20]
- **Progress**: 
    - sudo apt-get install ros-humble-imu-tools

### [2025-05-10]
- **Progress**

    Running al system
    ```bash
    ros2 launch jolie_bringup real_robot.launch.py

    docker run -it --rm --net=host --device=/dev/ttyACM0 microros/micro-ros-agent:humble serial --dev /dev/ttyACM0

    ros2 launch jolie_localization local_localization.launch.py

    ros2 launch jolie_mapping slam.launch.py
    ```
