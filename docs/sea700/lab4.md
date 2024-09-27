# Lab 4 : Gazebo and JetAuto

<font size="5">
Seneca Polytechnic</br>
SEA700 Robotics for Software Engineers
</font>

## Introduction

### URDF

URDF (Unified Robot Description Format) is an XML format for representing a robot model. URDF is commonly used in Robot Operating System (ROS) tools such as rviz (Ros Visualization tool) and Gazebo simulator. It is essentially a 3-D model with information around joints, motors, mass, etc. The files are then run through the Robot Operating System (ROS). The data from the file informs the human operator what the robot looks like and is capable of before they begin operating the robot.

More details on the URDF specification can be found [here](http://wiki.ros.org/urdf/XML).

Many robotic manufacturers have URDF models of their devices available for download. These include the [Segway RMP](https://stanleyinnovation.com/rmp-urdf-3d-files/), [Turtlebot](http://wiki.ros.org/turtlebot_description), and [AR10](https://github.com/Active8Robots/AR10/blob/master/ar10_description/urdf/ar10.urdf).

More Models:

[Agility Robotics Digit](https://github.com/adubredu/DigitRobot.jl)
[ANYbotics Anymal](https://github.com/ANYbotics/anymal_b_simple_description)
[Boston Dynamics Spot (via Clearpath ROS Driver)](https://github.com/clearpathrobotics/spot_ros)
[Clearpath Jackal](https://www.clearpathrobotics.com/assets/guides/kinetic/jackal/description.html)
[Clearpath Dingo](https://www.clearpathrobotics.com/assets/guides/melodic/dingo/description.html)
[Clearpath Husky](https://github.com/husky/husky)
[Clearpath Turtlebot](http://wiki.ros.org/turtlebot_description)
[Unitree Go1](https://github.com/unitreerobotics/unitree_ros)
[Universal Robotics](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)

### RViz

RViz is a 3D visualizer for the Robot Operating System (ROS) framework.

### Gazebo

## Preparation

In preperation of using the JetAuto robot, please be familiar with the user manual and the basic lesson provided by the manufacturer found here:

- [JetAuto User Manual](JetAuto-User-Manual.pdf)
- [JetAuto & JetAuto Pro Resources](https://drive.google.com/drive/folders/16pwHYO8rK-22oAzStc7-olP9Weq7AbzY)

### Building a Simple robot model

### SSH Into the JetAuto Robot

**Copy this lab instruction somewhere on your computer as you'll lose connection to the internet!**

1. By default, the JetAuto is configured to be in AP mode. Power on the robot and connect to the robot's WiFi starting in "HW-". The password for the WiFi connection is: **hiwonder**.

1. Once connected, use terminal (or PuTTY) to SSH into the robot at "192.168.149.1". The user is: **Jetauto**, and the password is: **hiwonder**.

### Configuring ROS 2

To be able to communicate our simulation with ROS 2 you need to use a package called ros_gz_bridge. This package provides a network bridge which enables the exchange of messages between ROS 2 and Gazebo Transport. You can install this package by typing:

        sudo apt-get install ros-humble-ros-ign-bridge

https://gazebosim.org/docs/latest/ros_installation/

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

https://gazebosim.org/docs/fortress/install/

Screen into Jetson Nano
https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#setup-headless

sudo apt-get install -y screen
sudo screen /dev/ttyACM0 115200

login: jetauto
password: hiwonder

hi

sudo apt install ros-humble-desktop

sudo apt install ros-humble-urdf (not required)
sudo apt install ros-humble-xacro

ros-melodic-

## Reference

- [ROS 2 Documentation: Humble](https://docs.ros.org/en/humble/index.html)
- [What is the Unified Robotics Description Format (URDF)?](https://formant.io/resources/glossary/urdf/)
- EECS 106A Labs