#!/bin/bash

# Config defaults
ros_domain=0
ros_type=full
ros_distro=none
is_jetson=false
build_realsense_from_source=false

# For testing, only runs selected parts of the script
install_ros=true
uninstall_ros=false
install_realsense=true
uninstall_realsense=false
install_realsense_ros=true
uninstall_realsense_ros=false

# Get Ubuntu Distro
release=$(lsb_release -cs)  # Get codename

# Get Kernel
kernel=$(uname -r)

SUB='tegra'
if [[ "$kernel" == *"$SUB"* ]]; then
	echo "Device is a Jetson."
	is_jetson=true
else
	echo "Device is not a Jetson."
fi

if [ $install_ros == true ]; then
	echo "Will install ROS."
fi

if [ $install_realsense == true  ]; then
	echo "Will install realsense."
fi

if [ $build_realsense_from_source == true  ]; then
	echo "Will build realsense from source."
fi

# Import ROS config
source ros.config

# Working Directory
cwd=$PWD

# Display the Config info
echo "Currernt working directory:" $cwd
echo "ROS Domain:" $ros_domain
echo "ROS Type:" $ros_type


# Select ROS distro based on OS distro
# The T/F statements can be changed to select which version of ROS to use
if [ $release == "focal" ]; then
	if true; then
       	echo "OS is Ubuntu 20.04. Using ROS Galactic."
       	ros_distro="galactic"
       else
       	echo "OS is Ubuntu 20.04. Using ROS Foxy."
       	ros_distro="foxy"
       fi
       	
elif [ $release == "bionic" ]; then
	
	if false; then
		echo "OS is Ubuntu 18.04. Using ROS Eloquent."
		ros_distro="eloquent"
	else
		echo "OS is Ubuntu 18.04. Using ROS Dashing."
		ros_distro="dashing"
	fi

else
       echo "OS is not a compatible version. Exiting."
       exit 1
fi



# S************ Auto-Install ROS ******************



if [  $install_ros == true  ]; then

	#locale  # check for UTF-8
	sudo apt update && sudo apt install locales -y
	sudo locale-gen en_US en_US.UTF-8
	sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
	export LANG=en_US.UTF-8
	locale  # verify settings
	
	rosdep_install="python_rosdep"
	
	# Different distros will have different sourcing methods and packages
	if [ $ros_distro=="galactic" ]; then
		rosdep_install="python3-rosdep2"
		sudo apt install software-properties-common
		sudo add-apt-repository universe
		sudo apt update && sudo apt install -y curl gnupg lsb-release
		sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
		echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
	elif [ $ros_distro=="foxy" ]; then
		rosdep_install="python3-rosdep2"
		sudo apt update && sudo apt install -y curl gnupg2 lsb-release
		sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
		echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
	elif [ $ros_distro=="eloquent" ]; then
		sudo apt update && sudo apt install -y curl gnupg2 lsb-release
		curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
		sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
	elif [ $ros_distro=="dashing" ]; then
		sudo apt update && sudo apt install -y curl gnupg2 lsb-release
		sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
		echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
	fi


	# Install ROS

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

	sudo apt install -y ${ros_install}

	sudo apt install -y python3-pip
	pip3 install -U argcomplete

	# Additional stuff
	sudo apt install -y rsync ${rosdep_install} python3-colcon-common-extensions git
	sudo rosdep init
	rosdep update

	
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

elif [ $uninstall_ros == true ]; then
	sudo apt remove ros-$ros_distro-* && sudo apt autoremove
fi


# ******** Install Intel Realsense SDK and tools ********


# Install the Realsense SDK
if [ $install_realsense == true ]; then

	sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
	sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
	sudo apt install -y librealsense2-dkms librealsense2-utils
	sudo apt install -y librealsense2-dev librealsense2-dbg

elif [ $uninstall_realsense == true  ]; then
	dpkg -l | grep "realsense" | cut -d " " -f 3 | xargs sudo dpkg --purge
fi


# Install the ROS wrapper for Realsense
if [ $install_realsense_ros == true ]; then
	# ROS Realsense Dependencies
	sudo apt install -y ros-$ros_distro-realsense2-camera
	sudo apt install -y ros-$ros_distro-realsense2-description
fi


if false; then
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

fi

