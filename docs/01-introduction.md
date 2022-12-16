---
layout: page
title: Introduction
permalink: /introduction/
---
Welcome! This is the website for our E205 final project, FastSLAM 1.0 for MR.CLAM

# Motivation
Simultaneous Localization and Mapping (SLAM) is used situations where both the robot path and its environment are uncertain. For example, SLAM can be useful in locations with unreliable GPS information, such as underwater and underground environments. Through SLAM, we are able to simultaneously map the environment and track a robot's location. 

FastSLAM 1.0 is a landmark based SLAM like the EKF SLAM we learned about in class, and it only works in non-dynamic scenes.

<br>
# Problem Statement
For our project, we aimed to localize a robot's position in unknown environment using FastSLAM 1.0.

#### <u>Input Measurements</u>
Our algorithm takes in an input of forward velocity, v [m/s], and angular velocity, ω [rad/s] to propagate particle states.

To perform particle reweights and updates, we use a robot's measurement data, which specifies its distance, r [m], and bearing, φ [rad], from an obstacle. More information on this can be found in the <b>[Data](https://echen4628.github.io/fastslam1/data/)</b> section.

#### <u>State Variables Being Estimated</u>
We defined stated variables for both the robot and landmarks. Each particle has a 3x1 matrix representing the robot's state variables, a 15x2 matrix representing the landmarks' state variables, and a 15x2x2 matrix representing the landmarks' covariance matrices. 

<div>
  <img src="https://echen4628.github.io/fastslam1/assets/img/Particle.png" alt="Particle" width="400" />
</div>

###### <b>Fig. 1</b> Breakdown of variables in a single particle

A description of the variables being estimated is below:

Robot States:
$x$: Robot's x-position [m]<br>
$y$: Robot's y-position [m]<br>
$\theta$: Robot's yaw position [radians]<br>

Landmark States (represented as a 2x1 matrix for individual landmarks):
$L_x$: Landmark's x-position [m]<br>
$L_y$: Landmark's y-position [m]<br>

Landmark Covariance (represented as a 2x2 matrix for individual landmarks):
$\sigma_{L_x}^2$: Landmark's xx covariance<br>
$\sigma_{L_x L_y}$: Landmark's xy covariance<br>
$\sigma_{L_y L_x}$: Landmark's yx covariance<br>
$\sigma_{L_y}^2$: Landmark's yy covariance<br>

#### <u>Intermediate Variables</u>
The intermediate values used to compute the state variables are described in detail in the <b>[Method](https://echen4628.github.io/fastslam1/method/)</b> section.

#### <u>Coordinate Frames</u>
Our project uses two coordinate frames: the inertial frame and the robot's frame. Both frames use a coordinate system where positive-x is to the right, positive-y is upwards, and positive yaw angle direction is counterclockwise. The robot's frame is rotated with respect to the inertial frame by the robot's yaw, θ. The figure below shows the relationship between the coordinate frames.

<div>
  <img src="https://echen4628.github.io/fastslam1/assets/img/Bearing_Frames.png" alt="Bearing Frames" width="400" />
</div>

###### <b>Fig. 2</b> Robot frame vs inertial frame, where θ is the robot's yaw 