# This repository holds all the development work related to the KouBot Project done by me.

# Table of Contents
 - [About](#about)
    - [Project Goals](#project-goals)
    - [Key Features](#key-features)
    - [Power System](#power-system)
 - [Teleoperation](#teleoperation)
 - [Mapping](#mapping)
 - [Localization](#teleoplocalizationeration)
 - [Autonomous Navigation](#autonomous-navigation)
 - [Next](#next)

# About 

<div align="center">
  <img src="doc/0-teleop_cropped.gif" alt="base" width="400"/>
</div>

<br>

Welcome to the <em>Koubot</em> project repository! <em>Koubot</em>  is an ambitious project aimed at developing an autonomous mobile robot equipped with **four mecanum wheels**, each powered by its own motor. This unique wheel configuration allows for omnidirectional movement, making <em>Koubot</em> highly maneuverable and capable of navigating complex environments with ease.

<br>


## Project Goal

Design and build an affordable, ROS 2-based indoor robot with full autonomous navigation capabilities, leveraging Docker for reproducibility and portability.


## Key Features


* **Computer**: Raspberry Pi Model B.

* **Software**: ROS 2 Galactic, Docker container, Gazebo, RViz.

* **Sensors**: LiDAR (RPLIDAR-A1M8 by Slamtec), Depth camera (OAK-D Lite), IMU (Adafruit - BNO055).

* **Mecanum Wheel Configuration**: Each of the 4 wheels is driven by its own motor, enabling omnidirectional movement for versatile navigation.

* **Sensor Fusion**: Integrates data from IMU and wheel encoders to improve odometry and overall navigation accuracy.



## Power System



![power_system](doc/power_diagram_full.png)

<br>

The power system of <em>Koubot</em> is divided into two sections to ensure efficient and reliable operation:

* **Motor Power Supply**: This section is dedicated to powering the 4 motors that drive the mecanum wheels, providing the necessary torque and control for movement.

* **System Power Supply**: This section powers the computer and sensors, ensuring stable and continuous operation of the robot's processing and sensing capabilities.

# Teleoperation

<div align="center">
  <img src="doc/1-teleop.gif" alt="base"/>
</div>

*KouBot* supports manual control through a Bluetooth-connected PS4 controller, allowing for smooth and precise holonomic movement in all directions. This setup was essential for early testing, calibration, and validating motor functionality before implementing full autonomy.

# Mapping

<div align="center">
  <img src="doc/2-mapping_x5.gif" alt="base"/>
</div>

Using SLAM Toolbox, KouBot was able to construct accurate 2D maps of its environment in real time.

Here is the resulting map:

<div align="center">
  <img src="doc/map.png" alt="base" width="400" />
</div>

# Localization

<div align="center">
  <img src="doc/3-localization_x5.gif" alt="base"/>
</div>

*KouBot* currently performs localization using LiDAR-based scan matching. In the future, IMU data will be fused with LiDAR-derived odometry using an Extended Kalman Filter (EKF) to enhance robustness and accuracy.


# Autonomous Navigation

<div align="center">
  <img src="doc/4-AN_x5.gif" alt="base"/>
</div>

*KouBot* uses the ROS 2 Nav2 stack to navigate point-to-point, avoiding obstacles and reaching goals with high precision.

# Next

While KouBot has proven its core SLAM and navigation capabilities, future improvements will focus on:

* Integrating full-state localization using an Extended Kalman Filter (EKF) to fuse LiDAR and IMU data.
* Planning to integrate a depth camera (OAK-D) for object detection and image processing experiments.
* Refining the motion controller with feedback from wheel encoders.