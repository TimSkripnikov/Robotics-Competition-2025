# Robotics Competition 2025

This repository contains the core ROS2 system developed for an autonomous driving competition within a university robotics course.
The project implements a modular, node-based architecture for controlling a mobile robot in a simulated autorace scenario.

The system covers the competition pipeline:
- starting on a green traffic light (using OpenCV)
- lane following
- intersection handling based on visual signs (using OpenCV)
- obstacle-related behavior
- finishing the race autonomously


## Dependencies
```
sudo apt install ros-jazzy-tf-transformations
pip3 install numpy==1.26
```

## To build:
```
colcon build
```

```
source your_folder/install/setup.bash
```

## To launch:
Terminal 1:

```
ros2 launch robot_bringup autorace_2025.launch.py
```

Terminal 2:

```
ros2 launch autorace_core_ArchieRobotics autorace_core.launch.py
```

Terminal 3:

```
ros2 run referee_console mission_autorace_2025_referee
```
