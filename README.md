# Problem Description
* Control the Franka Panda robot to move from position A to position B. You need to use inverse kinematics to solve for the joint space values. Then apply the trajectory planning method such that the robot can move smoothly for this task. Basically, you can follow this website to complete this task: https://erdalpekel.de/?p=55
* (not done) Control the Franka Panda robot such that the robot end-effector can move on a ball with an arbitrary radius. To be more specific, the robot end-effector position is on the ball surface and the z-axis of its end effector is perpendicular to the ball surface. If the robot is not able to cover the completed ball surface with the desired position & orientation, it is fine to control the robot to cover part of the ball surface.

# Restrictions
 feel free to select the ROS versions

# Output result
* send the video
* the C++ code. I forked from repo from website https://github.com/erdalpekel/panda_simulation.git  and added my code into the following repo https://github.com/nosmokingsurfer/panda_simulation. All the code is in the node called [robot_ab_control_node.cpp](https://github.com/nosmokingsurfer/panda_simulation/blob/master/src/robot_ab_control_node.cpp)
* Plot of joint value for motor 1-7 in both tasks
* Task space position error in x-, y-, and z- direction in both tasks
* Task space orientation error in task 2


# My setup:
* Ubuntu 18.04
* ROS Melodic + catkin tools + moveit http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html#install-ros-and-catkin
* VisualCode IDE + compile_command.json used for Intellisense syntax highlighting and autocompletion
* Libfranca and franca_ros - build from source. https://frankaemika.github.io/docs/installation_linux.html
* Pabda_simulation repo was forked from repo: https://github.com/erdalpekel/panda_simulation
* The catkin workspace was configured as follows:
![catkin](assets/catkin_config.png)

## To run the experiment:
```
roslaunch panda_simulation task1_ab_control.launch
```
In RViz window you can press `Next` or `Continue` buttons like in the following video:

<a href="http://www.youtube.com/watch?feature=player_embedded&v=_5eIY0zyKI0
" target="_blank"><img src="http://img.youtube.com/vi/_5eIY0zyKI0/0.jpg" 
alt="experiment video" width="640" height="480" border="10" /></a>

Experiment contains 5 random points from task space and drives the panda_arm to home position in the end;

## Plots
To plot the plots the `errors.csv` file required - it is automatically generated from experiment. There is one I've commited for example.
To plot use the following command:
```
python plot.py
```
The following figures should appear:

![task1](assets/task_1_joints.png)

![task1](assets/task_1_errors.png)


# Down below is the original instruction for panda_simulation package
---------------------------------------------------------------------

# panda_simulation

![Panda in Gazebo](assets/panda-in-gazebo.png?raw=true "Panda in Gazebo")

This package was written for ROS melodic running under Ubuntu 18.04. Run the following commands to make sure that all additional packages are installed:

```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/erdalpekel/panda_simulation.git
git clone https://github.com/erdalpekel/panda_moveit_config.git
git clone --branch simulation https://github.com/erdalpekel/franka_ros.git
cd ..
sudo apt-get install libboost-filesystem-dev
rosdep install --from-paths src --ignore-src -y --skip-keys libfranka
cd ..
```
It is also important that you build the *libfranka* library from source and pass its directory to *catkin_make*  when building this ROS package as described in [this tutorial](https://frankaemika.github.io/docs/installation.html#building-from-source).

Currently it includes a controller parameter config file and a launch file to launch the [Gazebo](http://gazebosim.org) simulation environment and the Panda robot from FRANKA EMIKA in it with the necessary controllers.

Build the catkin workspace and run the simulation:
```
catkin_make -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
source devel/setup.bash
roslaunch panda_simulation simulation.launch
```

Depending on your operating systems language you might need to export the numeric type so that rviz can read the floating point numbers in the robot model correctly:

```
export LC_NUMERIC="en_US.UTF-8"
```
Otherwise, the robot will appear in rviz in a collapsed state.


You can see the full explanation in my [blog post](https://erdalpekel.de/?p=55).

## Changelog:

   [_MoveIt!_ constraint-aware planning](https://erdalpekel.de/?p=123)

   This repository was extended with a ROS node that communicates with the _MoveIt!_ Planning Scene API. It makes sure that the motion planning pipeline avoids collision objects in the environment specified by the user in a separate directory (`~/.panda_simulation`) as _json_ files.

   [Publishing a box at Panda's hand in _Gazebo_](https://erdalpekel.de/?p=123)

   This repository was extended with a node that publishes a simple box object in the _Gazebo_ simulation at the hand of the robot. The position of this box will get updated as soon as the robot moves.

   [Visual Studio Code Remote Docker](https://erdalpekel.de/?p=123)

   I have added configuration files and additional setup scripts for developing and using this ROS package within a *Docker* container. Currently user interfaces for Gazebo and RViz are not supported.

   [Position based trajectory execution](https://erdalpekel.de/?p=285)

   The joint specifications in Gazebo were changed from an effort interface to position based interface. Furthermore, the PID controller was substituted with the simple gazebo internal position based control mechanism for a more stable movement profile of the robot. A custom joint position based controller was implemented in order to set the initial joint states of the robot to a valid configuration.

   [Automatic robot state initialization](https://erdalpekel.de/?p=314)

   A separate ROS node was implemented that starts a custom joint position controller and initializes the robot with a specific configuration. It switches back to the default controllers after the robot reaches the desired state.

![Panda state initialization in Gazebo](assets/robot-state-initializer.gif?raw=true "Panda state initialization in Gazebo")
