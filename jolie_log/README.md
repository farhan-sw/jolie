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
    ```
- **Issues**: 
    - The device is not general, if the device port changes, the docker command will not work. Need to find a way to make it general. e.g. /dev/ttyUSB0 -> esp32_base

- **Next Steps**: 
    - Make an OOP for the main C file, and integrate with the current base code
    - Make a folder contain libraries
