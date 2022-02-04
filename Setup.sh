#!/bin/bash

ros_domain=10

source ros.config

cwd=$PWD

echo "Currernt working directory:" $cwd


release=$(lsb_release -cs)  # Get codename
ros_version="none"


if [ $release == "bionic" ]; then
       echo "OS is Ubuntu 18.04. Using ROS2 Eloquent."
       ros_version="eloquent"
else
       echo "OS is not a compatible version. Exiting."
       exit 1
fi


# Auto Install ROS

locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings

sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update


ros_install=ros-$ros_version-desktop
# ros_install=ros-$ros_version-ros-base
pip_install=python-pip
rsync_install=rsync

sudo apt install ${ros_install} ${pip_install} ${rsync_install} 




# Check ~/.bashrc for sourcing

if grep -q "source /opt/ros/${ros_version}/setup.bash" ~/.bashrc; then
    echo "~/.bashrc line 1 found"
else
    echo -e "\e[93m~/.bashrc line NOT found. Adding line to EOF.\e[39m"
    echo "source /opt/ros/${ros_version}/setup.bash" >> ~/.bashrc
fi


# Check ~/.bashrc for ROS Domain


if grep -q "source /opt/ros/${ros_version}/setup.bash$" ~/.bashrc; then
    echo "~/.bashrc line 1 found"
else
    echo -e "\e[93m~/.bashrc line NOT found. Adding line to EOF.\e[39m"
    echo "source /opt/ros/${ros_version}/setup.bash" >> ~/.bashrc
fi


# Check ~/.bashrc for ROS Domain

if grep -q "export ROS_DOMAIN=$ros_domain$" ~/.bashrc; then
    echo "~/.bashrc line 1 found"
else
    echo -e "\e[93m~/.bashrc line NOT found. Adding line to EOF.\e[39m"
    echo "export ROS_DOMAIN=$ros_domain" >> ~/.bashrc
fi


printenv | grep -i ROS_VERSION
printenv | grep -i ROS_DISTRO
printenv | grep -i ROS_LOCALHOST_ONLY
printenv | grep -i ROS_PYTHON_VERSION











mkdir -p ~/catkin_ws/src


if grep -q 'source ~/catkin_ws/devel/setup.bash' ~/.bashrc; then
    echo "~/.bashrc line 2 found"
else
    echo -e "\e[93m~/.bashrc line NOT found. Adding line to EOF.\e[39m"
    echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
fi


if grep -q 'hid_sensor_custom' /etc/modules; then
    echo "hid_sensor_custom found"
else
    echo -e "\e[93mhid_sensor_custom NOT found. Adding line to EOF.\e[39m"
    echo 'hid_sensor_custom' | sudo tee -a /etc/modules
fi



# Install some stuff needed.
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential



# Install Realsense SDK and dependencies from source.
sudo apt install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt install libglfw3-dev

file=$cwd/librealsense-2.37.0.tar.gz
if [ ! -f "$file" ]; then
    echo -e "\e[31m$file is missing in current working directory\e[39m"
    exit 1
else
    echo "librealsense: '$file'"
    tar -xf "$file" -C ~/Downloads
fi

cd ~/Downloads/librealsense-2.37.0
./scripts/setup_udev_rules.sh
./scripts/patch-realsense-ubuntu-lts.sh
mkdir build && cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true -DBUILD_WITH_TM2=ON
sudo make uninstall && make clean && make -j$(nproc) && sudo make install



# Copy packages
cd $cwd/catkin_ws/src/
for D in *; do
    if [ -d "${D}" ]; then
        echo "Package: ${D}"
        rsync -r -t -v --progress --delete -u -c -s $cwd/catkin_ws/src/${D}/ ~/catkin_ws/src/${D}
    fi
done



# Copy supporting packages
cd $cwd/supporting_packages/
for F in *.tar.gz; do
    if [ -f "${F}" ]; then
	echo "Supporting Package: ${F}"
	tar -xf "${F}" -C ~/catkin_ws/src
    fi
done



# Gazebo models for the Kinect
mkdir -p ~/.gazebo/models
file=$cwd/kinect_ros.tar.gz
if [ ! -f "$file" ]; then
    echo -e "\e[31m$file is missing in current working directory\e[39m"
    exit 1
else
    echo "Found '$file'"
    echo "Extracting to ~/.gazebo/models"
    tar -xf "$file" -C ~/.gazebo/models
fi



# Modification to the lidar to extend the range to that of the RPLidar (12 meters).
file=$cwd/hokuyo_04lx_laser.gazebo.xacro
if [ ! -f "$file" ]; then
    echo -e "\e[31m$file is missing in current working directory\e[39m"
    exit 1
else
    echo "Found '$file'"
    echo "Extracting to ~/catkin_ws/src/common-sensors/common_sensors/urdf/sensors/"
    cp "$file" ~/catkin_ws/src/common-sensors/common_sensors/urdf/sensors/
fi



# Kinect IR Calibration
file=$cwd/depth_B00364725109104B.yaml
if [ ! -f "$file" ]; then
    echo -e "\e[31m$file is missing in current working directory\e[39m"
    exit 1
else
    echo "Found '$file'"
    echo "Extracting to ~/.ros/camera_info/"
    cp "$file" ~/.ros/camera_info/
fi



# Kinect Camera Calibration
file=$cwd/rgb_B00364725109104B.yaml
if [ ! -f "$file" ]; then
    echo -e "\e[31m$file is missing in current working directory\e[39m"
    exit 1
else
    echo "Found '$file'"
    echo "Extracting to ~/.ros/camera_info/"
    cp "$file" ~/.ros/camera_info/
fi



# Auto Install Dependencies

rosdep install --from-paths ~/catkin_ws/src --ignore-src --rosdistro=$ros_version -y



# Build everything

source /opt/ros/$ros_version/setup.bash
cd ~/catkin_ws
catkin_make -j$(nproc)
source ~/catkin_ws/devel/setup.bash



# Check dependencies
cd ~/catkin_ws/src/
for D in *; do
    if [ -d "${D}" ]; then
        cd ~/catkin_ws/src/${D}
        roswtf
    fi
done
