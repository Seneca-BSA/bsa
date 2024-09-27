# Lab 3 : ROS Workspace, Package, Publisher and Subscriber

<font size="5">
Seneca Polytechnic</br>
SEA700 Robotics for Software Engineers
</font>

## Introduction

### ROS Workspace

A workspace is a directory containing ROS packages. Before using ROS, it’s necessary to source your ROS installation workspace in the terminal you plan to work in. This makes ROS’s packages available for you to use in that terminal.

### ROS Package

A package is an organizational unit for your ROS code. If you want to be able to install your code or share it with others, then you’ll need it organized in a package. With packages, you can release your ROS work and allow others to build and use it easily.

For a package to be considered a catkin package it must meet a few requirements:

- The package must contain a catkin compliant `package.xml` file.
    - That `package.xml` file provides meta information about the package.
- The package must contain a `CMakeLists.txt` which uses catkin.
    - If it is a catkin metapackage it must have the relevant boilerplate `CMakeLists.txt` file.
- Each package must have its own folder
    - This means no nested packages nor multiple packages sharing the same directory.

The simplest possible package may have a file structure that looks like:

    my_package/
        CMakeLists.txt
        package.xml

A single workspace can contain as many packages as you want, each in their own folder. You can also have packages of different build types in one workspace (CMake, Python, etc.). You cannot have nested packages.

Best practice is to have a `src` folder within your workspace, and to create your packages in there. This keeps the top level of the workspace “clean”.

A trivial workspace might look like:

    workspace_folder/        -- WORKSPACE
        src/                   -- SOURCE SPACE
            CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
            package_1/
                CMakeLists.txt     -- CMakeLists.txt file for package_1
                package.xml        -- Package manifest for package_1
            ...
            package_n/
                CMakeLists.txt     -- CMakeLists.txt file for package_n
                package.xml        -- Package manifest for package_n

## Procedures

### Create a Workspace directory

1. Best practice is to create a new directory for every new workspace. The name doesn’t matter, but it is helpful to have it indicate the purpose of the workspace. Let’s choose the directory name `ros_ws`, for “development workspace”. Open a new terminal and run:

        mkdir -p ~/ros_ws/src
        cd ~/catkin_ws/
        catkin_make
    
    The `catkin_make` command is a convenience tool for working with catkin workspaces. Running it the first time in your workspace, it will create a `CMakeLists.txt` link in your `src` folder.

    Another best practice is to put any packages in your workspace into the `src` directory. The above code creates a `src` directory inside `ros_ws`.

    If you are building ROS from source to achieve Python 3 compatibility, and have setup your system appropriately (ie: have the Python 3 versions of all the required ROS Python packages installed, such as catkin) the first catkin_make command in a clean catkin workspace must be:

        catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

    This will configure catkin_make with Python 3. You may then proceed to use just `catkin_make` for subsequent builds.

1. Additionally, if you look in your current directory you should now have a 'build' and 'devel' folder. Inside the 'devel' folder you can see that there are now several setup files. Sourcing any of these files will overlay this workspace on top of your environment. Before continuing source your new setup.sh file:

        source devel/setup.bash

1. To make sure your workspace is properly overlayed by the setup script, make sure `ROS_PACKAGE_PATH` environment variable includes the directory you're in.

        echo $ROS_PACKAGE_PATH
        
    You should see:
        
        /home/youruser/catkin_ws/src:/opt/ros/kinetic/share

### Create a C++ Package

1. Navigate into `ros_ws/src`, and run the package creation command to create a simple C++ publisher and subscriber:

        cd ~/ros_ws/src

        catkin_create_pkg cpp_pubsub std_msgs roscpp

    Your terminal will return a message verifying the creation of your package `cpp_pubsub` and all its necessary files and folders.

    `catkin_create_pkg` requires that you give it a package_name and optionally a list of dependencies on which that package depends: `catkin_create_pkg <package_name> [depend1] [depend2] [depend3]`

    #### Write the publisher node

1. Navigate into `ros_ws/src/cpp_pubsub/src`. This is the directory in any CMake package where the source files containing executables belong.

1. Download the example talker code by entering the following command:

        wget -O talker.cpp https://raw.githubusercontent.com/ros/ros_tutorials/kinetic-devel/roscpp_tutorials/talker/talker.cpp
    
1. Now there will be a new file named `talker.cpp`. Open the file using your preferred text editor. Alternatively, create a `.cpp` file with the following:

        #include "ros/ros.h"
        #include "std_msgs/String.h"

        #include <sstream>

        /**
         * This tutorial demonstrates simple sending of messages over the ROS system.
         */
        int main(int argc, char **argv)
        {
            /**
             * The ros::init() function needs to see argc and argv so that it can perform
             * any ROS arguments and name remapping that were provided at the command line.
             * For programmatic remappings you can use a different version of init() which takes
             * remappings directly, but for most command-line programs, passing argc and argv is
             * the easiest way to do it.  The third argument to init() is the name of the node.
             *
             * You must call one of the versions of ros::init() before using any other
             * part of the ROS system.
             */
            ros::init(argc, argv, "talker");

            /**
             * NodeHandle is the main access point to communications with the ROS system.
             * The first NodeHandle constructed will fully initialize this node, and the last
             * NodeHandle destructed will close down the node.
             */
            ros::NodeHandle n;

            /**
             * The advertise() function is how you tell ROS that you want to
             * publish on a given topic name. This invokes a call to the ROS
             * master node, which keeps a registry of who is publishing and who
             * is subscribing. After this advertise() call is made, the master
             * node will notify anyone who is trying to subscribe to this topic name,
             * and they will in turn negotiate a peer-to-peer connection with this
             * node.  advertise() returns a Publisher object which allows you to
             * publish messages on that topic through a call to publish().  Once
             * all copies of the returned Publisher object are destroyed, the topic
             * will be automatically unadvertised.
             *
             * The second parameter to advertise() is the size of the message queue
             * used for publishing messages.  If messages are published more quickly
             * than we can send them, the number here specifies how many messages to
             * buffer up before throwing some away.
             */
            ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

            ros::Rate loop_rate(10);

            /**
             * A count of how many messages we have sent. This is used to create
             * a unique string for each message.
             */
            int count = 0;
            while (ros::ok())
            {
                /**
                 * This is a message object. You stuff it with data, and then publish it.
                 */
                std_msgs::String msg;

                std::stringstream ss;
                ss << "hello world " << count;
                msg.data = ss.str();

                ROS_INFO("%s", msg.data.c_str());

                /**
                 * The publish() function is how you send messages. The parameter
                 * is the message object. The type of this object must agree with the type
                 * given as a template parameter to the advertise<>() call, as was done
                 * in the constructor above.
                 */
                chatter_pub.publish(msg);

                ros::spinOnce();

                loop_rate.sleep();
                ++count;
            }

            return 0;
        }
    
    #### The Code Explained

    Now, let's break the code down.

        #include "ros/ros.h"

    `ros/ros.h` is a convenience include that includes all the headers necessary to use the most common public pieces of the ROS system.

        #include "std_msgs/String.h"

    This includes the `std_msgs/String` message, which resides in the `std_msgs` package. This is a header generated automatically from the `String.msg` file in that package. For more information on message definitions, see the [msg](https://wiki.ros.org/msg) page.

        ros::init(argc, argv, "talker");

    Initialize ROS. This allows ROS to do name remapping through the command line -- not important for now. This is also where we specify the name of our node. Node names must be unique in a running system.

    The name used here must be a base name, ie. it cannot have a / in it.

        ros::NodeHandle n;

    Create a handle to this process' node. The first `NodeHandle` created will actually do the initialization of the node, and the last one destructed will cleanup any resources the node was using.

        ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    Tell the master that we are going to be publishing a message of type `std_msgs/String` on the topic `chatter`. This lets the master tell any nodes listening on `chatter` that we are going to publish data on that topic. The second argument is the size of our publishing queue. In this case if we are publishing too quickly it will buffer up a maximum of 1000 messages before beginning to throw away old ones.

    `NodeHandle::advertise()` returns a `ros::Publisher` object, which serves two purposes: 1) it contains a `publish()` method that lets you publish messages onto the topic it was created with, and 2) when it goes out of scope, it will automatically unadvertise.

        ros::Rate loop_rate(10);

    A `ros::Rate` object allows you to specify a frequency that you would like to loop at. It will keep track of how long it has been since the last call to `Rate::sleep()`, and sleep for the correct amount of time.

    In this case we tell it we want to run at 10Hz.

        int count = 0;
        while (ros::ok())
        {

    By default roscpp will install a SIGINT handler which provides Ctrl-C handling which will cause `ros::ok()` to return false if that happens.

    `ros::ok()` will return false if:
    - a SIGINT is received (Ctrl-C)
    - we have been kicked off the network by another node with the same name
    - `ros::shutdown()` has been called by another part of the application.
    - all `ros::NodeHandles` have been destroyed

    Once ros::ok() returns false, all ROS calls will fail.

        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

    We broadcast a message on ROS using a message-adapted class, generally generated from a msg file. More complicated datatypes are possible, but for now we're going to use the standard `String` message, which has one member: "data".

        chatter_pub.publish(msg);

    Now we actually broadcast the message to anyone who is connected.

        ROS_INFO("%s", msg.data.c_str());

    `ROS_INFO` and friends are our replacement for `printf/cout`. See the [rosconsole documentation](https://wiki.ros.org/rosconsole) for more information.

        ros::spinOnce();

    Calling `ros::spinOnce()` here is not necessary for this simple program, because we are not receiving any callbacks. However, if you were to add a subscription into this application, and did not have `ros::spinOnce()` here, your callbacks would never get called. So, add it for good measure.

        loop_rate.sleep();

    Now we use the `ros::Rate` object to sleep for the time remaining to let us hit our 10Hz publish rate.
    
    Here's the condensed version of what's going on:
    - Initialize the ROS system
    - Advertise that we are going to be publishing std_msgs/String messages on the chatter topic to the master
    - Loop while publishing messages to chatter 10 times a second

    Now we need to write a node to receive the messsages.

    #### Write the subscriber node

1. Return to `ros_ws/src/cpp_pubsub/src` to create the next node. Enter the following code in your terminal to download the subscriber:

        wget -O listener.cpp https://raw.github.com/ros/ros_tutorials/kinetic-devel/roscpp_tutorials/listener/listener.cpp

    Check to ensure that these files exist:

        talker.cpp  listener.cpp

1. Open the `listener.cpp` with your text editor. Alternatively, create a `.cpp` file with the following:

        #include "ros/ros.h"
        #include "std_msgs/String.h"

        /**
         * This tutorial demonstrates simple receipt of messages over the ROS system.
         */
        void chatterCallback(const std_msgs::String::ConstPtr& msg)
        {
            ROS_INFO("I heard: [%s]", msg->data.c_str());
        }

        int main(int argc, char **argv)
        {
            /**
             * The ros::init() function needs to see argc and argv so that it can perform
             * any ROS arguments and name remapping that were provided at the command line.
             * For programmatic remappings you can use a different version of init() which takes
             * remappings directly, but for most command-line programs, passing argc and argv is
             * the easiest way to do it.  The third argument to init() is the name of the node.
             *
             * You must call one of the versions of ros::init() before using any other
             * part of the ROS system.
             */
            ros::init(argc, argv, "listener");

            /**
             * NodeHandle is the main access point to communications with the ROS system.
             * The first NodeHandle constructed will fully initialize this node, and the last
             * NodeHandle destructed will close down the node.
             */
            ros::NodeHandle n;

            /**
             * The subscribe() call is how you tell ROS that you want to receive messages
             * on a given topic.  This invokes a call to the ROS
             * master node, which keeps a registry of who is publishing and who
             * is subscribing.  Messages are passed to a callback function, here
             * called chatterCallback.  subscribe() returns a Subscriber object that you
             * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
             * object go out of scope, this callback will automatically be unsubscribed from
             * this topic.
             *
             * The second parameter to the subscribe() function is the size of the message
             * queue.  If messages are arriving faster than they are being processed, this
             * is the number of messages that will be buffered up before beginning to throw
             * away the oldest ones.
             */
            ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

            /**
             * ros::spin() will enter a loop, pumping callbacks.  With this version, all
             * callbacks will be called from within this thread (the main one).  ros::spin()
             * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
             */
            ros::spin();

            return 0;
        }

    #### The Code Explained

    Now, let's break it down piece by piece, ignoring some pieces that have already been explained above.

        void chatterCallback(const std_msgs::String::ConstPtr& msg)
        {
            ROS_INFO("I heard: [%s]", msg->data.c_str());
        }

    This is the callback function that will get called when a new message has arrived on the `chatte`r topic. The message is passed in a `boost shared_pt`r, which means you can store it off if you want, without worrying about it getting deleted underneath you, and without copying the underlying data.

        ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    Subscribe to the `chatter` topic with the master. ROS will call the `chatterCallback()` function whenever a new message arrives. The 2nd argument is the queue size, in case we are not able to process messages fast enough. In this case, if the queue reaches 1000 messages, we will start throwing away old messages as new ones arrive.

    `NodeHandle::subscribe()` returns a `ros::Subscriber` object, that you must hold on to until you want to unsubscribe. When the Subscriber object is destructed, it will automatically unsubscribe from the `chatter` topic.

    There are versions of the `NodeHandle::subscribe()` function which allow you to specify a class member function, or even anything callable by a `Boost.Function` object. The [roscpp](https://wiki.ros.org/roscpp/Overview) overview contains more information.

        ros::spin();
    
    `ros::spin()` enters a loop, calling message callbacks as fast as possible. Don't worry though, if there's nothing for it to do it won't use much CPU. `ros::spin()` will exit once `ros::ok()` returns false, which means `ros::shutdown()` has been called, either by the default Ctrl-C handler, the master telling us to shutdown, or it being called manually.

    Again, here's a condensed version of what's going on:

    - Initialize the ROS system
    - Subscribe to the chatter topic
    - Spin, waiting for messages to arrive
    - When a message arrives, the chatterCallback() function is called

### Build and Run C++ Package

1. Now, open up `CMakeLists.txt` and ensure the following are in there.

        ## Declare ROS messages and services
        add_message_files(FILES Num.msg)
        add_service_files(FILES AddTwoInts.srv)

        ## Generate added messages and services
        generate_messages(DEPENDENCIES std_msgs)

        ## Declare a catkin package
        catkin_package()

        ## Build talker and listener
        include_directories(include ${catkin_INCLUDE_DIRS})

        add_executable(talker src/talker.cpp)
        target_link_libraries(talker ${catkin_LIBRARIES})
        add_dependencies(talker beginner_tutorials_generate_messages_cpp)

        add_executable(listener src/listener.cpp)
        target_link_libraries(listener ${catkin_LIBRARIES})
        add_dependencies(listener beginner_tutorials_generate_messages_cpp)

1. This will create two executables, `talker` and `listener`, which by default will go into package directory of your `devel` space, located by default at `~/catkin_ws/devel/lib/<package name>`.

    Note that you have to add dependencies for the executable targets to message generation targets:

        add_dependencies(talker beginner_tutorials_generate_messages_cpp)

    This makes sure message headers of this package are generated before being used. If you use messages from other packages inside your catkin workspace, you need to add dependencies to their respective generation targets as well, because catkin builds all projects in parallel. As of *Groovy* you can use the following variable to depend on all necessary targets:

        target_link_libraries(talker ${catkin_LIBRARIES})

    You can invoke executables directly or you can use `rosrun` to invoke them. They are not placed in `<prefix>/bin` because that would pollute the PATH when installing your package to the system. If you wish for your executable to be on the PATH at installation time, you can setup an install target, see: [catkin/CMakeLists.txt](https://wiki.ros.org/catkin/CMakeLists.txt)

1. Now run `catkin_make` in your catkin workspace:

        cd ~/catkin_ws
        catkin_make

    Note: Or if you're adding as new pkg, you may need to tell catkin to force making by `--force-cmake` option.

Now you have written a simple publisher and subscriber.

1. Source the setup files:

        . install/setup.bash

1. Now run the talker node from `ros2_ws`:

        ros2 run cpp_pubsub talker

    The terminal should start publishing info messages every 0.5 seconds, like so:

        [INFO] [minimal_publisher]: Publishing: "Hello World: 0"
        [INFO] [minimal_publisher]: Publishing: "Hello World: 1"
        [INFO] [minimal_publisher]: Publishing: "Hello World: 2"
        [INFO] [minimal_publisher]: Publishing: "Hello World: 3"
        [INFO] [minimal_publisher]: Publishing: "Hello World: 4"

1. Open another terminal, source the setup files from inside `ros2_ws` again, and then start the listener node:

        . install/setup.bash

        ros2 run cpp_pubsub listener

    The listener will start printing messages to the console, starting at whatever message count the publisher is on at that time, like so:

        [INFO] [minimal_subscriber]: I heard: "Hello World: 10"
        [INFO] [minimal_subscriber]: I heard: "Hello World: 11"
        [INFO] [minimal_subscriber]: I heard: "Hello World: 12"
        [INFO] [minimal_subscriber]: I heard: "Hello World: 13"
        [INFO] [minimal_subscriber]: I heard: "Hello World: 14"

1. Enter Ctrl+C in each terminal to stop the nodes from spinning.

### Create a Python Package

1. Navigate into `ros2_ws/src`, and run the package creation command to create a simple Python publisher and subscriber:

        ros2 pkg create --build-type ament_python --license Apache-2.0 py_pubsub

    Your terminal will return a message verifying the creation of your package `py_pubsub` and all its necessary files and folders.

    #### Write the publisher node

1. Navigate into `ros2_ws/src/py_pubsub/py_pubsub`. This directory is a Python package with the same name as the ROS 2 package it’s nested in.

1. Download the example talker code by entering the following command:

        wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py

1. Now there will be a new file named `publisher_member_function.py` adjacent to `__init__.py`. Open the file using your preferred text editor. Alternatively, create a .py file with the following:

        import rclpy
        from rclpy.node import Node

        from std_msgs.msg import String


        class MinimalPublisher(Node):

            def __init__(self):
                super().__init__('minimal_publisher')
                self.publisher_ = self.create_publisher(String, 'topic', 10)
                timer_period = 0.5  # seconds
                self.timer = self.create_timer(timer_period, self.timer_callback)
                self.i = 0

            def timer_callback(self):
                msg = String()
                msg.data = 'Hello World: %d' % self.i
                self.publisher_.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg.data)
                self.i += 1


        def main(args=None):
            rclpy.init(args=args)

            minimal_publisher = MinimalPublisher()

            rclpy.spin(minimal_publisher)

            # Destroy the node explicitly
            # (optional - otherwise it will be done automatically
            # when the garbage collector destroys the node object)
            minimal_publisher.destroy_node()
            rclpy.shutdown()


        if __name__ == '__main__':
            main()

    The first lines of code after the comments import `rclpy` so its `Node` class can be used.

        import rclpy
        from rclpy.node import Node

    The next statement imports the built-in string message type that the node uses to structure the data that it passes on the topic.

        from std_msgs.msg import String

    These lines represent the node’s dependencies. Recall that dependencies have to be added to `package.xml`, which you’ll do in the next section.

    Next, the `MinimalPublisher` class is created, which inherits from (or is a subclass of) `Node`.

        class MinimalPublisher(Node):

    Following is the definition of the class’s constructor. `super().__init__` calls the `Node` class’s constructor and gives it your node name, in this case `minimal_publisher`.

    `create_publisher` declares that the node publishes messages of type `String` (imported from the std_msgs.msg module), over a topic named `topic`, and that the “queue size” is 10. Queue size is a required QoS (quality of service) setting that limits the amount of queued messages if a subscriber is not receiving them fast enough.

    Next, a timer is created with a callback to execute every 0.5 seconds. `self.i` is a counter used in the callback.

        def __init__(self):
            super().__init__('minimal_publisher')
            self.publisher_ = self.create_publisher(String, 'topic', 10)
            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.i = 0

    `timer_callback` creates a message with the counter value appended, and publishes it to the console with `get_logger().info`.

        def timer_callback(self):
            msg = String()
            msg.data = 'Hello World: %d' % self.i
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1

    Lastly, the main function is defined.

        def main(args=None):
            rclpy.init(args=args)

            minimal_publisher = MinimalPublisher()

            rclpy.spin(minimal_publisher)

            # Destroy the node explicitly
            # (optional - otherwise it will be done automatically
            # when the garbage collector destroys the node object)
            minimal_publisher.destroy_node()
            rclpy.shutdown()

    First the `rclpy` library is initialized, then the node is created, and then it “spins” the node so its callbacks are called.

    #### Add dependencies

1. Navigate one level back to the `ros2_ws/src/py_pubsub` directory, where the `setup.py`, `setup.cfg`, and `package.xml` files have been created for you. Open `package.xml` with your text editor and make sure to fill in the `<description>`, `<maintainer>` and `<license>` tags:

        <description>Examples of minimal publisher/subscriber using rclpy</description>
        <maintainer email="you@email.com">Your Name</maintainer>
        <license>Apache License 2.0</license>

1. After the lines above, add the following dependencies corresponding to your node’s import statements:

        <exec_depend>rclpy</exec_depend>
        <exec_depend>std_msgs</exec_depend>

    This declares the package needs `rclpy` and `std_msgs` when its code is executed.

    Make sure to save the file.

    #### Add an entry point

1. Open the `setup.py` file. Again, match the `maintainer`, `maintainer_email`, `description` and `license` fields to your `package.xml`:

        maintainer='YourName',
        maintainer_email='you@email.com',
        description='Examples of minimal publisher/subscriber using rclpy',
        license='Apache License 2.0',

1. Add the following line within the `console_scripts` brackets of the `entry_points` field:

        entry_points={
                'console_scripts': [
                        'talker = py_pubsub.publisher_member_function:main',
                ],
        },

    Don’t forget to save.

    #### Check setup.cfg

1. The contents of the `setup.cfg` file should be correctly populated automatically, like so:

        [develop]
        script_dir=$base/lib/py_pubsub
        [install]
        install_scripts=$base/lib/py_pubsub

    This is simply telling setuptools to put your executables in `lib`, because `ros2 run` will look for them there.

    You could build your package now, source the local setup files, and run it, but let’s create the subscriber node first so you can see the full system at work.

    #### Write the subscriber node

1. Return to `ros2_ws/src/py_pubsub/py_pubsub` to create the next node. Enter the following code in your terminal:

        wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py

    Now the directory should have these files:

        __init__.py  publisher_member_function.py  subscriber_member_function.py

1. Open the `subscriber_member_function.py` with your text editor. Alternatively, create a .py file with the following:

        import rclpy
        from rclpy.node import Node

        from std_msgs.msg import String


        class MinimalSubscriber(Node):

            def __init__(self):
                super().__init__('minimal_subscriber')
                self.subscription = self.create_subscription(
                    String,
                    'topic',
                    self.listener_callback,
                    10)
                self.subscription  # prevent unused variable warning

            def listener_callback(self, msg):
                self.get_logger().info('I heard: "%s"' % msg.data)


        def main(args=None):
            rclpy.init(args=args)

            minimal_subscriber = MinimalSubscriber()

            rclpy.spin(minimal_subscriber)

            # Destroy the node explicitly
            # (optional - otherwise it will be done automatically
            # when the garbage collector destroys the node object)
            minimal_subscriber.destroy_node()
            rclpy.shutdown()


        if __name__ == '__main__':
            main()

    The subscriber node’s code is nearly identical to the publisher’s. The constructor creates a subscriber with the same arguments as the publisher. Recall from earlier lab that the topic name and message type used by the publisher and subscriber must match to allow them to communicate.

        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

    The subscriber’s constructor and callback don’t include any timer definition, because it doesn’t need one. Its callback gets called as soon as it receives a message.

    The callback definition simply prints an info message to the console, along with the data it received. Recall that the publisher defines `msg.data = 'Hello World: %d' % self.i`

        def listener_callback(self, msg):
            self.get_logger().info('I heard: "%s"' % msg.data)

    The `main` definition is almost exactly the same, replacing the creation and spinning of the publisher with the subscriber.

        minimal_subscriber = MinimalSubscriber()

        rclpy.spin(minimal_subscriber)

    Since this node has the same dependencies as the publisher, there’s nothing new to add to `package.xml`. The `setup.cfg` file can also remain untouched.

    #### Add an entry point

1. Reopen `setup.py` and add the entry point for the subscriber node below the publisher’s entry point. The `entry_points` field should now look like this:

        entry_points={
                'console_scripts': [
                        'talker = py_pubsub.publisher_member_function:main',
                        'listener = py_pubsub.subscriber_member_function:main',
                ],
        },

    Make sure to save the file, and then your pub/sub system should be ready.

### Build and run Python Package

1. It’s good practice to run rosdep in the root of your workspace (`ros2_ws`) to check for missing dependencies before building:

        rosdep install -i --from-path src --rosdistro humble -y

    If the `rosdep` command is not found, run the following to install, init, and update `rosdep`:

        pip install -U rosdep
        rosdep init
        rosdep update

    If you get a permission denied error, use `sudo`.

    When all dependencies are met, `rosdep` will return a success message:

        #All required rosdeps installed successfully

1. Still in the root of your workspace, `ros2_ws`, build your new package:

        colcon build --packages-select py_pubsub

    If you want to build all the packages, just run `colcon build` without the option.

1. Open a new terminal, navigate to `ros2_ws`, and source the setup files:

        source install/setup.bash

1. Now run the talker node:

        ros2 run py_pubsub talker

    The terminal should start publishing info messages every 0.5 seconds, like so:

        [INFO] [minimal_publisher]: Publishing: "Hello World: 0"
        [INFO] [minimal_publisher]: Publishing: "Hello World: 1"
        [INFO] [minimal_publisher]: Publishing: "Hello World: 2"
        [INFO] [minimal_publisher]: Publishing: "Hello World: 3"
        [INFO] [minimal_publisher]: Publishing: "Hello World: 4"
        ...

1. Open another terminal, source the setup files from inside `ros2_ws` again, and then start the listener node:

        ros2 run py_pubsub listener

    The listener will start printing messages to the console, starting at whatever message count the publisher is on at that time, like so:

        [INFO] [minimal_subscriber]: I heard: "Hello World: 10"
        [INFO] [minimal_subscriber]: I heard: "Hello World: 11"
        [INFO] [minimal_subscriber]: I heard: "Hello World: 12"
        [INFO] [minimal_subscriber]: I heard: "Hello World: 13"
        [INFO] [minimal_subscriber]: I heard: "Hello World: 14"

1. Stop the listener and try to run the C++ listener from earlier:

        ros2 run cpp_pubsub listener

    You should see a similar same output.

1. Enter `Ctrl+C` in each terminal to stop the nodes from spinning.

### Install Gazebo

After being familiar with ROS, we'll now install the Gazebo simulation environment.

1. Following the instruction to [install Gazebo Fortress](https://gazebosim.org/docs/fortress/install_ubuntu/). Each Gazebo version works with a specific version of ROS.

1. Once installed, start Gazebo with the following command to ensure it's functional:

        ign gazebo

    ![Figure 3.1 Gazebo Quick Start](lab3-gazebo.png)

    ***Figure 3.1** Gazebo Quick Start*

1. Try to start one of the senario from the quick start screen.

1. Afterward, open another terminal and run the following command to see all the topics:

        ign topic -l

    Do you notice any similarity?

    We won't be able to connect ROS directly with Gazebo yet. That will be the topic of Lab 4 and 5.

## Lab Question

1. Write a new controller (C++ or Python) for turtlesim that replace `turtle_teleop_key`. Since the turtlesim node is the subscriber in this example, you’ll only need
to write a single publisher node.

    Create a new package called `lab3_turtlesim`. You can create a new workspace called `lab3_ws` or use your existing workspace.

        ros2 pkg create --build-type ament_cmake --license Apache-2.0 lab3_turtlesim --dependencies rclcpp geometry_msgs
    or

        ros2 pkg create --build-type ament_python --license Apache-2.0 lab3_turtlesim --dependencies rclpy geometry_msgs

    Your node should do the following:

    - Accept a command line argument specifying the name of the turtle it should control.
        - Running `ros2 run lab3_turtlesim turtle_controller turtle1` will start a controller node that controls turtle1.
    - Use `w`, `a`, `s`, `d` to control the turtle by publish velocity control messages on the appropriate topic whenever the user presses those keys on the keyboard, as in the original `turtle_teleop_key`. Capturing individual keystrokes from the terminal is slightly complicated, so feel free to use keyboard input such as `scanf()` or `input()` instead.

    **Hint:** You'll need to use the `Twist` message type in the `geometry_msgs` package.
    
    To test, spawn multiple turtles and open multiple instances of your new turtle controller node, each linked to a different turtle.

Once you've completed all the above steps, ask the lab professor or instructor over and demostrate that you've completed the lab and written down all your observations. You might be asked to explain some of the concepts you've learned in this lab.

## Reference

- [ROS 2 Documentation: Humble](https://docs.ros.org/en/humble/index.html)
- EECS 106A Labs