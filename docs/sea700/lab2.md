### Creating ROS Workspaces and Packages

Next, we'll also need to install `colcon` so we can create ROS package.
    
```
pip install -U colcon-common-extensions
```

First, we first have to create a ROS workspace – a directory used to organize and manage ROS packages. Since all ROS code must be contained within a package in a workspace, this is something you’ll do each time you start a new project.

#### Creating a Workspace

1. First, create a directory called `ros2_ws` and another directory called `lab1` within it.

    <pre>
        mkdir -p ~/ros2_ws/lab1
        cd ~/ros2_ws/lab1
    </pre>

1. Next, create a directory called `src` within `lab1`.

    <pre>
        mkdir src
        cd src
    </pre>

#### Creating a Package

1. From the `src` directory, let's create a package called `foo`.

    <pre>
        ros2 pkg create --build-type ament_cmake --license Apache-2.0 foo
    </pre>

1. Open your newly created package `foo` and examine the `package.xml` file. The only dependency in it should be `<buildtool_depend>ament_cmake</buildtool_depend>`.

1. Let's create another package and add some dependencies to it.

    ```
        ros2 pkg create --build-type ament_cmake --license Apache-2.0 bar --dependencies rclpy rclcpp std_msgs geometry_msgs turtlesim
    ```

1. Open your newly created package `bar` and examine the `package.xml` file. There should be additional dependency now.

#### Understanding ROS nodes

1. Once you’ve added all your resources to the new package, the last step before you can use the package with ROS is to build it. Run the `colcon build` command from the `lab1` directory. If everything worked, you should get a success notification.

### Creating ROS Workspaces and Packages

A quick review of some computation graph concepts:

- Node: an executable representing an individual ROS software process
- Message: a ROS datatype used to exchange data between nodes
- Topic: nodes can publish messages to a topic and/or subscribe to a topic to receive messages

We’re now ready to test out some actual software running on ROS.