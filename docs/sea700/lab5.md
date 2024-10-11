# Lab 5 : Robotic Arm and SLAM

<font size="5">
Seneca Polytechnic</br>
SEA700 Robotics for Software Engineers
</font>

## Forward Kinematics

As discussed in lecture, the forward kinematics problem involves fnding the confguration of a specifed link in a
robotic manipulator relative to some other reference frame, given the angles of each of the joints in the manipulator.

### Denavit–Hartenberg (DH) Parameters

In mechanical engineering, the Denavit–Hartenberg parameters (also called DH parameters) are the four parameters associated with a particular convention for attaching reference frames to the links of a spatial kinematic chain, or robot manipulator. In this convention, coordinate frames are attached to the joints between two links such that one transformation is associated with the joint \([Z]\), and the second is associated with the link \([X]\). The coordinate transformations along a serial robot consisting of n links form the kinematics equations of the robot:

$$
[T] = [Z_1][X_1][Z_2][X_2]...[Z_{n-1}][X_{n-1}][Z_n][X_n]
$$

where \([T]\) is the transformation that characterizes the location and orientation of the end-link.

![Figure 5.1 DH Kinematics Links](lab5-dh-kinematics-links.png)

***Figure 5.1** DH Kinematics Links*

As shown in the figure above, each joints (i-1, i, and i+1) has a unique line \(S\) (shown as dotted line in the figure above) in space that forms the joint axis and define the relative movement of its two links. For each sequence of lines \(S_i\) and \(S+{i+1}\), there is a common normal line \(A_{i, i+1}\). By convention, z-coordinate axes are assigned to the joint axes \(S_i\) and x-coordinate axes are assigned to the common normals \(A_{i, i+1}\).

> #### Four Parameters

> The following four transformation parameters (labelled in red text in the figure above) are known as DH parameters:

>   - d: offset along \(S_i\) (z-direciton) to the next common normal \(A_{i, i+1}\) (x-direction)
>   - θ: angle about \(S_i\) from old common normal \(A_{i-1, i}\) to the new common normal \(A_{i, i+1}\)
>   - a (or r): length of the common normal from \(S_i\) to \(S_{i+1}\)
>   - α: angle about common normal \(A_{i-1, i}\), from old axis \(S_i\) to new axis \(S_{i+1}\)

This convention allows the definition of the movement of links around a common joint axis \(S_i\) by the screw displacement:

$$
[Z_i] =
\begin{bmatrix}
   a & b \\
   c & d
\end{bmatrix}
$$

![Figure 5.3 Gazebo Running](lab5-jetauto-kinematics.png)

***Figure 5.3** Gazebo Running*

sudo apt install ros-melodic-moveit
sudo apt install ros-melodic-trac-ik-kinematics-plugin



sudo apt install ros-melodic-slam-gmapping


roslaunch jetauto_moveit_config demo_gazebo.launch false_execution:=false

roslaunch jetauto_gazebo room_worlds.launch
roslaunch jetauto_slam slam.launch sim:=true
roslunach jetsuto_slam rviz_slam.launch sim:=true


## Reference

- [Denavit–Hartenberg parameters](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters)
- [ROS Tutorials](https://wiki.ros.org/ROS/Tutorials)
- EECS 106A Labs