# JAM-L: Joint Arm Mobile Locomotor

A differential-drive mobile manipulator developed as part of the University of Sheffield ELE221 Mechatronics Robot Challenge.

## Overview

JAM-L combines autonomous navigation with robotic manipulation. The robot uses wheel encoders and PI control to navigate a predefined route before using a 2-DOF robotic arm to perform a drawing task on a whiteboard.

The project integrates mechanical design, embedded systems, control engineering, sensor fusion, and robotic kinematics into a single autonomous platform.

## Features

* Differential drive mobile robot
* Full quadrature encoder feedback
* PI position and heading control
* Straight-line drift correction
* Infrared sensor assisted final positioning
* 2-DOF robotic arm
* Inverse kinematics based arm control
* Custom geared parallel-jaw gripper
* Battery monitoring and LED status indication
* Finite-state-machine control architecture

## Hardware

### Controller

* Arduino Mega 2560

### Sensors

* Sharp GP2Y0A21YK0F IR Distance Sensor
* Dual Quadrature Wheel Encoders

### Actuators

* 2 × DFRobot FIT0450 Geared DC Motors
* 2 × MG996R Servos
* 1 × SG90 Servo

### Motor Driver

* TB6612FNG Dual H-Bridge

### Power

* 6 × 1.2V NiMH Cells (7.2V nominal)

## Control System

### Position Control

Forward motion is controlled using a PI controller operating on encoder-derived position measurements.

### Heading Control

Turning manoeuvres use a PI controller based on differential wheel encoder measurements.

### Straight-Line Correction

A secondary PI loop continuously compensates for wheel mismatch and drift.

### Final Approach

The robot switches from encoder-based navigation to infrared distance sensing when approaching the target whiteboard to eliminate accumulated positioning error.

## Robotic Arm

The arm consists of:

* Shoulder Joint (MG996R)
* Elbow Joint (MG996R)
* Geared Parallel-Jaw Gripper (SG90)

Inverse kinematics are used to calculate joint angles from Cartesian target coordinates.

## Mechanical Design

The robot features:

* Two-tier 3D printed chassis
* Differential drive locomotion
* Custom servo mounts
* Custom arm links
* Custom geared gripper mechanism
* Front caster support system

All custom components were designed in Autodesk Fusion and manufactured using FDM 3D printing.

## Results

The final robot successfully:

1. Navigated autonomously using encoder feedback.
2. Corrected final positioning using an IR sensor.
3. Executed an inverse-kinematics-based arm sequence.
4. Completed the required whiteboard drawing task.

## Authors

* Rotimi Dayo
* Valour Inyang
* Rustamkhon Ozodkhonzoda
* Arjun Rajesh

University of Sheffield
ELE221 Robot Challenge
