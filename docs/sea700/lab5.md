# Lab 5 : Robotic Arm and SLAM

<font size="5">
Seneca Polytechnic</br>
SEA700 Robotics for Software Engineers
</font>




sudo apt install ros-melodic-moveit
sudo apt install ros-melodic-trac-ik-kinematics-plugin


sudo apt install ros-melodic-slam-gmapping

roslaunch jetauto_gazebo room_worlds.launch
roslaunch jetauto_slam slam.launch sim:=true
roslunach jetsuto_slam rviz_slam.launch sim:=true

roslaunch jetauto_peripherals imu.launch
rostopic echo /imu_data

roslaunch jetauto_controller odom_publish.launch
rostopic echo /odom_raw



<!--

sudo apt-get install ros-melodic-slam-gmapping


-->