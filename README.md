# FRC-ROS2

This is the source code for using ROS2 with an FRC Robot. This is the successor to FRC-ROS and has been completely rebuilt from the ground up. The steps for installing and running ROS applications are identical for desktop PC's, laptops, Raspberry PI, NVidia Jetson, and other devices.

## Youtube Demos
Simulation Demo: https://www.youtube.com/watch?v=qSjgmY7hYsY

Teleop Demo w/ SLAM: https://www.youtube.com/watch?v=1Qe-s1liH5k

Autonomous and Swerve Simulation: https://youtu.be/OrnWRP20IEY

## Setup

Setup is fairly simple. You will need a computer with Ubuntu 18.04 installed. 18.04 was chosen because that was the the latest version of Ubuntu that is officially supported on the Jetson TX2. This package is intended to be used on a Jetson TX2, however, you should be able to run it on a Jetson Nano, Raspberry Pi, or some other single board computer. You will need to use another installation method to get that working.

### 1. Install Ubuntu on Your Computer

Make sure you have Ubuntu 18.04 installed on your computer: https://ubuntu.com/tutorials/install-ubuntu-desktop-1804#1-overview

### 2. Download FRC-ROS2 to Your Computer

Enter the following to download FRC-ROS to your Documents:
```
cd ~/Documents
git clone https://github.com/JakeAllison/FRC-ROS2.git
cd FRC-ROS2
./Setup.sh
```

### 3. Install NVidia Jetpack (Skip if you're not using a Jetson)

This part will be performed on your computer with the Jetson hooked up.
You will need to install the NVidia SDK manager and flash the Jetson. 
The SDK Manager can be downloaded from here: https://developer.nvidia.com/embedded/downloads

This repo is designed to be used with Jetpack 4.6 (rev 3), but other versions should work as long as you are using. You will need an NVidia developer's account to access this. When the file is downloaded, run the following commands to install the SDK Manager:

```
cd ~/Documents/FRC-ROS2
dpkg -i ./other_installs/sdkmanager_1.6.1-8175_amd64.deb
```

You can follow the guide (NVidia Developer membership required): https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson/index.html. At a minimum, you will want follow directions to flash the Jetson with a new OS. Installing anything else on either the host computer or the Jetson is optional.

### 4. Install FRC-ROS2 on Your Jetson (Skip if you're not using a Jetson)

Follow the steps above to install on your computer. This process is identical to your computer. Once you know how to use ROS, you will be able to connect your computer to your Jetson and have then interact seamlessly.

### 5. Make Updates When Needed

If you make changes to the source code, you will need to update the files from the location where you downloaded FRC-ROS to your Catkin workspace. Open a terminal and put the following command to bring up the simulation:

```
cd ~/Documents/FRC-ROS2
./Update.sh
```

### 6. Run a Simulation


You can run a robot simulation in Gazebo simulator. Open a terminal and put the following command to bring up the simulation:
```
roslaunch frc_robot main_sim.launch
```
1. You can control the virtual robot using the GUI screen that pops up, or an XBox 360 controller.
2. To control the robot, use the analog sticks on your controller. You need to hold down the **RB** button for normal speed, or **LB** for turbo speed.
2. You can set navigation goals in RViz using '2D Nav Goal' and placing the arrow anywhere you like. The robot will try to drive autonomously to that location.
3. You can hit **ctrl-c** in the terminal to exit.

The options include the following and can be typed in similar to above:
- auton: Enables autonomous nagivation and driving.
- kinectlidar: Starts up a Kinect and RPLidarA2 connected over USB.
- robot: FOR THE REAL ROBOT. This brings up all sensors and the bridge to the roboRIO.
- sim: Brings up a simulation.
- slam: Enables SLAM (Simeltaneous Localization and Mapping)
- teleop: Enables teleop control through either GUI or any joystick such as an XBox controller.
- user_interface: Brings up GUI for viewing and sending data to ROS.
- visualize: Bring up RViz used to visualize ROS data.

## To-Do

### Implement a Network Tables Interface for RoboRIO

This requires 2 parts. The first part is a ROS package capable of communicating over FRC Network Tables. The second part needs a software interface on the RoboRIO to send and receive data to and from the Jetson.

Interface will be like this:

RoboRio Software (RoboRio) <--Network Tables--> ROS Converter Node (Jetson) <-- ROS Topics --> ROS Software (Jetson)

### Implement Commands and Data Transfer

The following data is the most essential data that will be transferred:
- Velocity Commands (Jetson -> RoboRIO) (required)
- Send IMU measurements (RoboRIO -> Jetson) (optional)
- Send Wheel Odometry (RoboRIO -> Jetson) (optional)

The folowing data will allow for usability in competition:
- Send Commands (RoboRIO -> Jetson)
- Receive Status (Jetson -> RoboRIO)

Commands from RoboRIO -> Jetson could include stuff like:
- Set navigation goals.
- Activate target tracking.
- Switch to Mapping-Mode (SLAM) or Localization-Only (needs existing map)

Status from Jetson -> RoboRIO could include stuff like:
- Target tracking info
- Localization info
- Navigation statuses
- General ROS Status

### Implement Target Tracking

A single ROS node can take any camera data and perform image processing. This can be written and configured however the user likes and can output targeting data. A single node can be run, or multiple nodes can be doing processing on the same camera feed.

## Tutorials and Helpful Information

### Recommended Tutorials to Get Started

Getting Started:
- http://wiki.ros.org/
- http://wiki.ros.org/ROS/Tutorials

Additional Tutorials:
- http://wiki.ros.org/urdf/Tutorials
- http://gazebosim.org/tutorials?tut=ros_overview
- http://wiki.ros.org/rviz/Tutorials
- http://wiki.ros.org/navigation/Tutorials

### Packages and Software Used in FRC-ROS

Some sensor packages are included here:
- http://wiki.ros.org/rplidar
- http://wiki.ros.org/openni_launch

SLAM (Simeltaneous Localization and Mapping):
- https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping
- http://wiki.ros.org/hector_mapping
- http://wiki.ros.org/gmapping
- http://wiki.ros.org/rtabmap_ros

Robot Localization:
- https://roscon.ros.org/2015/presentations/robot_localization.pdf
- http://docs.ros.org/kinetic/api/robot_localization/html/index.html
- http://www.cs.cmu.edu/~rasc/Download/AMRobots5.pdf

Autonomous Navigation:
- https://en.wikipedia.org/wiki/Robot_navigation
- http://wiki.ros.org/move_base
- http://wiki.ros.org/costmap_2d
- http://wiki.ros.org/nav_core
- http://wiki.ros.org/navfn
- http://wiki.ros.org/dwa_local_planner

### Other Relevant Topics

Swerve Drive Kinematics:
- https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383

Machine learning is a more advanced topic:
- https://www.youtube.com/watch?v=aircAruvnKk
- https://developer.nvidia.com/embedded/twodaystoademo
- https://github.com/dusty-nv/ros_deep_learning

