# Automated Kiwifruit Harvester

This repository contains a ROS2 workspace for the automated kiwifruit harvesting algorithm, developed as part of a Summer Research Program. The project was designed to automate the harvesting process using a robotic arm and integrates with a RealSense D435i for fruit detection.

## System Requirements

- **Operating System**: Ubuntu Linux 22.04 with kernel `5.15.0-43-generic`
- **ROS2 Version**: Humble Hawksbill (up-to-date as of February 14, 2025)
- **Python Version**: 3.10.12
- **RealSense-ROS**: Installed from [Intel RealSense ROS GitHub](https://github.com/IntelRealSense/realsense-ros)  *(Can't find version, last update to GitHub files was September 3, 2024)*
- **ur_rtde**: *Version 1.6.0*, Installed from [UR RTDE Installation](https://sdurobotics.gitlab.io/ur_rtde/installation/installation.html)

## Connecting to the Robot

To connect to the UR5 robotic arm using `ur_rtde`, follow these steps:
1. Connect an ethernet cable between your computer and the robot.
2. Configure the IPv4 Wired Connection on your system:
   - **Method**: Manual
   - **Address**: `192.168.131.5`
   - **Netmask**: `255.255.255.0`
   - **Gateway**: `192.168.131.1`
3. Set correct robot IP address in code when connecting (For the UR5 used during testing this was 192.168.131.8)

## Running the System

To launch the system, use the following ROS2 launch command in the terminal:

```bash
ros2 launch srs_launch_files start_nodes_launch.py gripper_orientation:=(parameter default=basic) picking_method:=(parameter default=individual)
```
### Parameters
1. `picking_method:` Specify picking strategy for kiwifruit harvesting
   - **all**:    Detects once and tries to pick all detected kiwifruit
   - **individual**:    Picks one kiwifruit fruit and then redetects
2. `gripper_orientation` Defines the sorting and gripper orienting method used
   - **basic**:    Sorts from lowest to highest, does not change gripper orientation
   - **farthest_centroid**:   Pick farthest kiwifruit from cluster centroid, Orient gripper based on average Vector to neighbors
   - **z_order**:   Pick lowest kiwifruit first, orient gripper based on average neighbor vector
