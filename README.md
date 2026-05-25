# TurtleBot3 ROS2 Vision and Navigation Project

This project implements a ROS2-based TurtleBot3 robotic system for maze exploration, wall following, visual landmark detection, SLAM mapping, and waypoint navigation.

The project was developed using ROS2, Gazebo, RViz, Cartographer, Nav2, Python, and C++.

## Project Overview

The system contains two main stages:

### Stage 1: Wall Following, Mapping and Landmark Detection

In Stage 1, the TurtleBot3 robot explores a maze environment by following walls using LiDAR data. During exploration, the robot detects colored visual landmarks using camera images and publishes their estimated positions to RViz.

Main functions:

- Simulate TurtleBot3 in Gazebo
- Implement LiDAR-based wall following
- Detect colored landmarks from camera images
- Use HSV color segmentation for visual detection
- Transform detected landmark positions into the map frame
- Visualize landmarks using MarkerArray in RViz
- Build a map using Cartographer SLAM

### Stage 2: Map-based Navigation

In Stage 2, the robot uses the saved map from Stage 1 and performs autonomous waypoint navigation using Nav2.

Main functions:

- Load the saved map
- Initialize robot pose in RViz
- Use Nav2 for localization and path planning
- Navigate to detected landmark positions
- Execute waypoint-based navigation tasks

## Technologies Used

- ROS2
- TurtleBot3
- Gazebo
- RViz
- Cartographer
- Nav2
- Python
- C++
- OpenCV
- Linux
- tf2
- MarkerArray visualization

## Project Structure

```text
turtlebot3_ws/
├── src/
│   └── wall_follower/
│       ├── launch/
│       │   └── wall_follower.launch.py
│       ├── config/
│       │   └── waypoint_nav_params.yaml
│       ├── scripts/
│       │   ├── see_marker.py
│       │   ├── point_transformer.py
│       │   └── waypoint_navigator.py
│       ├── src/
│       │   └── wall_follower.cpp
│       ├── CMakeLists.txt
│       └── package.xml
