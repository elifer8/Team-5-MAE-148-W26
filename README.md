<p align="center">
  <img src="images/UCSDLogo_JSOE_BlueGold.png" width="500">
</p>

<h1 align="center">Team 5 - Traffic Control Obstacle Avoidance Car</h1>

<p align="center">
  <b>Institution:</b> University of California, San Diego (UCSD) <br>
  <b>School:</b> Jacobs School of Engineering (JSOE) <br>
  <b>Course:</b> MAE 148 - Autonomous Vehicles <br>
  <b>Term:</b> Winter 2026 <br>
  <b>Team:</b> Team 5
</p>

---

## Table of Contents
- [Table of Contents](#table-of-contents)
- [Team Members](#team-members)
- [Overview](#overview)
- [Goals We Met](#goals-we-met)
- [Final Project Videos](#final-project-videos)
- [CAD Parts](#cad-parts)
- [Key Files](#key-files)
- [Key Features](#key-features)
- [Final Assembly](#final-assembly)
- [Software Design](#software-design)
- [Gantt Chart](#gantt-chart)

---

## Team Members
| Name | Major |
|-----|-----|
| Azfar Syed | Electrical & Computer Engineering |
| Angel Chavez | Electrical & Computer Engineering |
| Juqy Chen | Electrical & Computer Engineering |
| Elisa Correa | Mechanical & Aerospace Engineering |


---

## Overview

This project was developed for MAE/ECE 148: Autonomous Vehicles at UC San Diego. The goal of the project was to design and implement a camera-based traffic control system for an autonomous RC vehicle that can navigate a track while responding to traffic signals and avoiding obstacles.

The system uses an OAK-D Lite camera and ROS2-based control architecture to detect environmental cues and adjust the vehicle’s behavior in real time. Computer vision models were trained using Roboflow to recognize traffic lights (red, yellow, and green) and orange traffic cones placed along the track. These detections are used to dynamically control the vehicle’s speed and navigation.

The vehicle performs lane-following and autonomous navigation while interpreting traffic signals to determine appropriate driving actions. For example, the vehicle adjusts its speed based on traffic light states and performs obstacle avoidance maneuvers when cones are detected.

To ensure reliable performance, the system integrates PID control for steering and speed regulation, and multiple camera configurations were tested to improve robustness under different lighting conditions. The system was evaluated by running autonomous laps on the course with increasing precision and stability.

This project demonstrates the integration of robot perception, computer vision, and control systems to create a responsive autonomous driving platform capable of reacting to real-world traffic scenarios.

---

## Goals We Met

- Successfully implemented a ROS2-based autonomous driving pipeline
- Developed and deployed a Roboflow model for traffic light detection (red, yellow, green)
- Integrated perception with control to enable stop/go behavior based on traffic light state

---

## Final Project Videos



---

## CAD Parts

<p align="center">
  <b>Camera Mount</b><br>
  <img src="images/4C765905-09F8-454A-8067-A0B70043C2E5.png" width="500">
</p>

<p align="center">
  <b>Bellypan</b><br>
  <img src="images/9AD5C9A3-BC3E-4858-A4CA-8338DD82F0AB.png" width="500">
</p>

<p align="center">
  <b>Raspberry Pi Mount</b><br>
  <img src="images/A24BB2DB-04E1-4D24-AE9A-6CAD14AC7077.png" width="500">
</p>

<p align="center">
  <b>Assembly</b><br>
  <img src="images/image.png" width="500">
</p>

---
## Key Files
We built upon the classes lane detection ROS2 implimentaion. Added files and changed:

Lane detection Node
- /home/projects/ros2_ws_a/src/ucsd_robocar_hub2/ucsd_robocar_lane_detection2_pkg/ucsd_robocar_lane_detection2_pkg/traffic_light_gate_node.py
- /home/projects/ros2_ws_a/src/ucsd_robocar_hub2/ucsd_robocar_lane_detection2_pkg/ucsd_robocar_lane_detection2_pkg/lane_guidance_node.py
- /home/projects/ros2_ws_a/src/ucsd_robocar_hub2/ucsd_robocar_lane_detection2_pkg/ucsd_robocar_lane_detection2_pkg/lane_guidance_node.py
- /home/projects/ros2_ws_a/src/ucsd_robocar_hub2/ucsd_robocar_lane_detection2_pkg/ucsd_robocar_lane_detection2_pkg/cone_depth_detector_node.py
- /home/projects/ros2_ws_a/src/ucsd_robocar_hub2/ucsd_robocar_lane_detection2_pkg/ucsd_robocar_lane_detection2_pkg/calibration_node.py
- /home/projects/ros2_ws_a/src/ucsd_robocar_hub2/ucsd_robocar_lane_detection2_pkg/config/ros_racer_calibration.yaml
- /home/projects/ros2_ws_a/src/ucsd_robocar_hub2/ucsd_robocar_lane_detection2_pkg/setup.py

sensor2 node
- /home/projects/ros2_ws_a/src/ucsd_robocar_hub2/ucsd_robocar_sensor2_pkg/ucsd_robocar_sensor2_pkg/oakd_shared_node.py
- /home/projects/ros2_ws_a/src/ucsd_robocar_hub2/ucsd_robocar_sensor2_pkg/launch/camera_oakd.launch.py


## Key Features

**Traffic Light Detection**  
Detects and classifies **red, yellow, and green traffic lights** using a computer vision model trained on Roboflow. The vehicle adjusts its speed and behavior based on the detected signal state.

**Cone Detection and Obstacle Avoidance**  
Identifies **orange traffic cones** placed on the track and performs lane-switching maneuvers to safely avoid obstacles while continuing along the course.

**Camera-Based Perception**  
Uses an **OAK-D Lite camera** with DepthAI to capture real-time visual data and run object detection models for traffic lights and cones.

**ROS2 System Architecture**  
Implements a modular **ROS2 framework** where perception, control, and decision-making nodes communicate using publishers and subscribers.

**Autonomous Navigation**  
The RC vehicle autonomously drives laps around the track while maintaining lane alignment and responding to detected traffic signals and obstacles.

**Adaptive PID Control**  
Uses **PID-based steering and speed control**, with multiple parameter configurations tested to maintain stable driving performance under different lighting conditions.

--

## Final Assembly

---

## Software Design


---

## Gantt Chart

<p align="center">
  <b>Initial Gantt Chart</b><br>
  <img src="images/Initial Gantt.png" width="500">
</p>

<p align="center">
  <b>Final Gantt Chart</b><br>
  <img src="images/FinalGantt.png" width="500">
</p>



---

**Last Updated:** March 13, 2026  
**Presentation Date:** [Insert date]
