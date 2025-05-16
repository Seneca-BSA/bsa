# Project 1 - Setting up ROS1 and TurtleSim

<font size="5">
Seneca Polytechnic</br>
AIG240 Robotics for Software Engineers
</font>

<font size="5">
**Due:** Sunday, Jun 1, 2025 at 11:59PM on Blackboard</br>
**Weight:** 10%
</font>

## Introduction

In this project, you will demonstrate your achievement in setting up a ROS1 environment and creating a simple node to control a robot using TurtleSim.

## Objectives

1. Set up the ROS1 environment.
2. Create a simple ROS node to control a robot.
3. Answer assessment questions.

**NOTE:** This project must be completed individually.

- Seneca Academic Integrity Policy
- Seneca Generative Artificial Intelligence (GenAI) Policy

### Step 1: Set Up Your ROS Environment

1. Install ROS1 as per the Lab 1 instructions.
2. Create your ROS workspace as per the Lab 3 instructions.

### Step 2: Create and Run a Simple ROS Node

Use open-source code, AI-generated code, or your own code to create a controller as described below (and in Lab 3) to make the robot move in TurtleSim.

Your node should do the following:

- Accept a command-line argument specifying the name of the turtle it should control.
    - For example, running `rosrun lab3_turtlesim turtle_controller turtle1` will start a controller node that controls `turtle1`.
- Use `w`, `a`, `s`, `d` keys to control the turtle by publishing velocity control messages on the appropriate topic whenever the user presses those keys on the keyboard, as in the original `turtle_teleop_key`. **The turtle should ONLY move when a key is pressed. When the key is released, the turtle should STOP moving.**
- The controller should be able to listen to multiple keys simultaneously. For example, if `w` and `a` are pressed, the turtle should move forward and turn left in a circular path. If the keys pressed are contradictory, there should be no movement.

**Hint:** You'll need to use the `Twist` message type in the `geometry_msgs` package.  
**Hint:** Refer to the `turtle_teleop_key` source code as a reference.

## Assessment Questions

1. What command did you use to create a ROS package?
2. Explain why and how you used ROS messages in your program.
3. Describe the steps to launch ROS, TurtleSim, and your ROS node simultaneously.
4. How do you verify that your ROS node is publishing messages correctly?

## Submission

1. A link to your project folder (e.g., on GitHub (private) or Google Drive) containing all the necessary files and code.
2. A video showing your code running in Gazebo and the robot moving.
3. A text file containing the answers to the questions for every group member.

### Late Submission Penalty

1. A 25% reduction from the full mark will be applied for every 24 hours the submission is late after the deadline.
