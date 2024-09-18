# Lab 2 : ROS Nodes, Topics, Services, Parameters, Actions

<font size="5">
Seneca Polytechnic</br>
SEA700 Robotics for Software Engineers
</font>

## Introduction

A overview of some computation graph concepts:
- **Node:** an executable representing an individual ROS software process
- **Topic:** nodes can publish messages to a topic and/or subscribe to a topic to receive messages
- **Message:** a ROS datatype used to exchange data between nodes

### Nodes in ROS

Each node in ROS should be responsible for a single, modular purpose, ie. controlling the wheel motors or publishing the sensor data from a laser range-finder. Each node can send and receive data from other nodes via topics, services, actions, or parameters. A full robotic system is comprised of many nodes working in concert.

![Figure 2.1 ROS Nodes Graph](lab2-nodes-graph.gif)

***Figure 2.1** ROS Nodes Graph*

### Topics in ROS

ROS 2 breaks complex systems down into many modular nodes. Topics are a vital element of the ROS graph that act as a bus for nodes to exchange messages.

![Figure 2.2 ROS Topic Graph](lab2-topics-graph-1.gif)

***Figure 2.2** ROS Topic Graph*

A node may publish data to any number of topics and simultaneously have subscriptions to any number of topics.

![Figure 2.3 ROS Topic Graph (One-to-Many)](lab2-topics-graph-2.gif)

***Figure 2.3** ROS Topic Graph (One-to-Many)*

Topics are one of the main ways in which data is moved between nodes and therefore between different parts of the system.

## Procedures

### Understanding ROS Nodes

1. Open a terminal to run turtlesim. The command `ros2 run` launches an executable from a package.

        ros2 run <package_name> <executable_name>
        
    We need the package name to be `turtlesim` and the executable name to be `turtlesim_node`.

        ros2 run turtlesim turtlesim_node

1. To find the node names, the `ros2 node list` can be used. `ros2 node list` will show you the names of all running nodes. This is especially useful when you want to interact with a node, or when you have a system running many nodes and need to keep track of them.

    Open a new terminal while `/turtlesim` is still running and enter the following command:

        ros2 node list
    
    The terminal will return the node name:

        /turtlesim

1. Open another new terminal and start the teleop node with the command:

        ros2 run turtlesim turtle_teleop_key

1. Return to the terminal where you ran `ros2 node list` and run it again. You will now see the names of two active nodes:

        /turtlesim
        /teleop_turtle

1. [Remapping](https://design.ros2.org/articles/ros_command_line_arguments.html#name-remapping-rules) allows you to reassign default node properties, like node name, topic names, service names, etc., to custom values. Let’s open another `/turtlesim` node and reassign the name to `/my_turtle`. In a new terminal, run the following command:

        ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle

1. Return to the terminal where you ran `ros2 node list`, and run it again, you will see three node names:

        /my_turtle
        /turtlesim
        /teleop_turtle

1. To access more information about a node, use the following command:

        ros2 node info <node_name>

    To examine your latest node, `my_turtle`, run the following command:

        ros2 node info /my_turtle

    `ros2 node info` returns a list of subscribers, publishers, services, and actions. i.e. the ROS graph connections that interact with that node. The output should look like this:

        /my_turtle
          Subscribers:
            /parameter_events: rcl_interfaces/msg/ParameterEvent
            /turtle1/cmd_vel: geometry_msgs/msg/Twist
          Publishers:
            /parameter_events: rcl_interfaces/msg/ParameterEvent
            /rosout: rcl_interfaces/msg/Log
            /turtle1/color_sensor: turtlesim/msg/Color
            /turtle1/pose: turtlesim/msg/Pose
          Service Servers:
            /clear: std_srvs/srv/Empty
            /kill: turtlesim/srv/Kill
            /my_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
            /my_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
            /my_turtle/get_parameters: rcl_interfaces/srv/GetParameters
            /my_turtle/list_parameters: rcl_interfaces/srv/ListParameters
            /my_turtle/set_parameters: rcl_interfaces/srv/SetParameters
            /my_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
            /reset: std_srvs/srv/Empty
            /spawn: turtlesim/srv/Spawn
            /turtle1/set_pen: turtlesim/srv/SetPen
            /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
            /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
          Service Clients:

          Action Servers:
            /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
          Action Clients:

1. Run the same comand on the `/teleop_turtle` node and see the difference.

### Understanding ROS Topics

1. Close the `/my_turtle` terminal so only the `/turtlesim` and `/teleop_turtle` are open.

1. We will use `rqt_graph` to visualize the changing nodes and topics, as well as the connections between them. Open a new terminal and enter the command:

        rqt_graph

### Complete the following tutorials

1. [Understanding topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
1. [Understanding services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
1. [Understanding parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
1. [Understanding actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
1. [Launching nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html)
1. [Recording and playing back data](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)

## Lab Question

1. Create a recording of a turtle performing a "Figure 8" path and play it back.

Once you've completed all the above steps, ask the lab professor or instructor over and demostrate that you've completed the lab and written down all your observations. You might be asked to explain some of the concepts you've learned in this lab.

## Reference

- [ROS 2 Documentation: Humble](https://docs.ros.org/en/humble/index.html)
- EECS 106A Labs