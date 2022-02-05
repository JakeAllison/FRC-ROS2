#!/bin/bash

ros_domain=0
ros_type=full
ros_distro=none
release=$(lsb_release -cs)  # Get codename
build_realsense_from_source=true

source ros.config

cwd=$PWD

echo "Currernt working directory:" $cwd
echo "ROS Domain:" $ros_domain
echo "ROS Type:" $ros_type


# Get OS Version
if [ $release == "focal" ]; then
       echo "OS is Ubuntu 20.04. Using ROS Noetic."
       ros_distro="noetic"
elif [ $release == "bionic" ]; then
       echo "OS is Ubuntu 18.04. Using ROS Melodic."
       ros_distro="eloquent"
else
       echo "OS is not a compatible version. Exiting."
       exit 1
fi


# Auto Install ROS

#locale  # check for UTF-8
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings

sudo apt update -y
sudo apt install curl gnupg2 lsb-release -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update -y

# Set if ROS will be a basic or full installation
if [ $ros_type == "basic" ]; then
	ros_install=ros-$ros_distro-ros-base
elif [ $ros_type == "full" ]; then
       ros_install=ros-$ros_distro-desktop
else
       echo "ros_type must be either 'basic' or 'full'. Check ros.config. Exiting."
       exit 1
fi



sudo apt install ${ros_install} -y
sudo apt install python-pip rsync python-rosdep python3-colcon-common-extensions git -y
sudo rosdep init
rosdep update







# Install Intel Realsense SDK and tools

if [ $build_realsense_from_source ]; then
	# Option #1: Build from source
	
	
	# Install prequisites and dependencies
	file=$cwd/other_installs/librealsense-2.50.0.zip
	if [ ! -f "$file" ]; then
	    echo -e "\e[31m$file is missing in current working directory\e[39m"
	    exit 1
	else
	    echo "librealsense: '$file'"
	    unzip "$file" -d ~/Downloads
	fi
	
	cd ~/Downloads/librealsense-2.50.0/
	sudo apt install git libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev -y
	
	if [ $release == "bionic" ]; then
		sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at -y
		./scripts/setup_udev_rules.sh
    		echo -e "\e[93mPatching kernel for libealsense. This will take a while.\e[39m"
		./scripts/patch-realsense-ubuntu-lts.sh
	else
		echo "OS version unsupported by librealsense source install. Exiting."
		exit 1
	fi
	

	# Build librealsense

	mkdir build && cd build

	# Full version includes examples. Both are optimized for release
	if [ $ros_type == "full" ]; then
		cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=false
	elif [ $ros_type == "basic" ]; then
		cmake ../ -DCMAKE_BUILD_TYPE=Release
	else
	       echo "ros_type must be either 'basic' or 'full'. Check ros.config. Exiting."
	       exit 1
	fi
	
	# Build nad install
	sudo make uninstall && make clean && make -j$(nproc) && sudo make install
else
	# Option #2: Premade Binaries
	sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
	sudo apt install librealsense2-dkms librealsense2-utils -y
	sudo apt install librealsense2-dev librealsense2-dbg -y
fi




# Check ~/.bashrc for sourcing

if grep -q "source /opt/ros/${ros_distro}/setup.bash$" ~/.bashrc; then
    echo "~/.bashrc line 1 found"
else
    echo -e "\e[93m~/.bashrc line NOT found. Adding line to EOF.\e[39m"
    echo "source /opt/ros/${ros_distro}/setup.bash" >> ~/.bashrc
fi


if grep -q "source ~/ros2_ws/install/local_setup.bash$" ~/.bashrc; then
    echo "~/.bashrc line 1 found"
else
    echo -e "\e[93m~/.bashrc line NOT found. Adding line to EOF.\e[39m"
    echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc
fi



# Check ~/.bashrc for ROS Domain

if grep -q "export ROS_DOMAIN=$ros_domain$" ~/.bashrc; then
    echo "~/.bashrc line 1 found"
else
    echo -e "\e[93m~/.bashrc line NOT found. Adding line to EOF.\e[39m"
    echo "export ROS_DOMAIN=$ros_domain" >> ~/.bashrc
fi



# Print ROS version information

printenv | grep -i ROS_VERSION
printenv | grep -i ROS_DISTRO
printenv | grep -i ROS_LOCALHOST_ONLY
printenv | grep -i ROS_PYTHON_VERSION



# Create workspace

mkdir -p ~/ros2_ws/src


# ROS Realsense Dependencies
sudo apt install ros-$ros_distro-realsense2-camera
sudo apt install ros-$ros_distro-realsense2-description



# Copy packages
cd $cwd/ros2_ws/src/
for D in *; do
    if [ -d "${D}" ]; then
        echo "Package: ${D}"
        rsync -r -t -v --progress --delete -u -c -s $cwd/ros2_ws/src/${D}/ ~/ros2_ws/src/${D}
    fi
done


z
# Copy supporting packages
cd $cwd/supporting_packages/
for F in *.tar.gz; do
    if [ -f "${F}" ]; then
	echo "Supporting Package: ${F}"
	tar -xf "${F}" -C ~/catkin_ws/src
    fi
done



# Install package dependencies and build

source /opt/ros/$ros_distro/setup.bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro $ros_distro -y
colcon build --symlink-install

