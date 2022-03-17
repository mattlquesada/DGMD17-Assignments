## UBUNTU MATE 18.04 INSTALLATION ON RASPBERRY PI

NOTE: Ubuntu MATE can run on Raspberry Pi 3B, 3B+ and model 4B with 2GB, 4GB or 8GB of RAM. Model 4B is recommended.

The installation is straight forward:

1. Download the Ubuntu MATE 18.04 for Raspberry Pi (ARM 64) from this link:
	https://releases.ubuntu-mate.org/archived/bionic/arm64/
	Pick the file: 'ubuntu-mate-18.04.2-beta1-desktop-arm64%2Braspi3-ext4.img.xz' or any newer version if available.

2. Burn the ISO file on a micro SD card Class 10 or better with at least 16GB.

3. Put the micro SD card in the Raspberry Pi, boot and follow the steps to install an configure Ubuntu (it's almost the same as when installing Ubuntu on a PC)

**NOTE**: If at any time you get a pop up window asking you to upgrade, with a message like this:

Ubuntu 20.04.1 LTS Upgrade Available
"A new version of Ubuntu is available. Would you like to upgrade?"

Pick "Don't Upgrade", or Ubuntu will be upgraded to version 20.04. Don't upgrade because we will be installing ROS melodic, wich is for Ubuntu 18.04. At the time of this writing the ROS Noetic version for Ubuntu 20.04 wasn't running well the 'cv_bridge' package used in the 'opencv_node.py' ROS node of the article's example.

--------------------------------------------------------------------------------
## UBUNTU MATE 18.04 ENABLE SSH ACCESS
To be able to connect remotely to the Raspberry Pi and control the robotic car from a remote PC, you must have the SSH server in the Raspberry Pi propperly configured.

Start SSH with this command:
	sudo systemctl start ssh

If you get the error: "Could not load host key...", regenerate the SSH host keys (see the procedure below).

I used the 'ubuntu-mate-18.04.2-beta1-desktop-arm64%2Braspi3-ext4.img.xz' Ubuntu 18.04 ISO for my Ubuntu MATE RPi installation. For some reason, the SSH keys in this image were missing. I did the following to create a new set of SSH keys and start propperly the SSH service:

(Based on: https://www.cyberciti.biz/faq/howto-regenerate-openssh-host-keys/ )

Regenerate OpenSSH Host Keys:

1. – Delete old ssh host keys:
    /bin/rm -v /etc/ssh/ssh_host_*

	If you get the following error:
    /bin/rm: cannot remove '/etc/ssh/ssh_host_*': No such file or directory

	Dismiss the error and go the next step.

2. Regenerate OpenSSH Host Keys:
    sudo dpkg-reconfigure openssh-server

3. Restart the SSH server:
    sudo systemctl restart ssh

4. Enable to automatically start SSH on boot:
    sudo systemctl enable ssh

--------------------------------------------------------------------------------
## UBUNTU 18.04 INSTALL OF ROS MELODIC

1. Configure your Ubuntu repositories

Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse." You can follow the Ubuntu guide for instructions on doing this.

(_This step is optional because in Ubuntu 18.04 those repositories are correctly configured by default_)

2. Setup your sources.list

Setup your computer to accept software from packages.ros.org.

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'


3. Set up your keys

    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

4. Installation

First, make sure your Debian package index is up-to-date:

    sudo apt update

Install just the ROS bare-bones version:

    4.2 ROS-Base: (Bare Bones) ROS package, build, and communication libraries. No GUI tools.

        sudo apt install ros-melodic-ros-base

5. Environment setup

It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched:

	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
	source ~/.bashrc

6. Dependencies for building packages

To install this tool and other dependencies for building ROS packages, run:

	sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

6.1 Initialize rosdep

	sudo apt install python-rosdep

With the following, you can initialize rosdep.

	sudo rosdep init
	rosdep update

--------------------------------------------------------------------------------
## UBUNTU CREATE A CATKIN WORKSPACE

1. Let's create and build a catkin workspace:

	mkdir -p ~/catkin_ws/src
	cd ~/catkin_ws/
	catkin_make

2. Source your new setup.*sh file:
	
	echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
	source ~/.bashrc

3. Make sure ROS_PACKAGE_PATH environment variable includes the directory you're in by executing:

	echo $ROS_PACKAGE_PATH

    You must get an output like this: /home/pi/catkin_ws/src:/opt/ros/melodic/share

--------------------------------------------------------------------------------
## INSTALLING ROS CV-BRIDGE PACKAGE
Run the command:

	sudo apt install ros-melodic-cv-bridge

--------------------------------------------------------------------------------
## CREATING AND RUNNING A CUSTOM ROS PACKAGE

# Create the 'robotic_car' custom ROS package. It will depend on the packages
# 'rospy', 'std_msgs', 'geometry_msgs' and 'sensor_msgs'
cd ~/catkin_ws/src
catkin_create_pkg robotic_car rospy std_msgs geometry_msgs sensor_msgs

# Create the scripts folder inside the package
mkdir robotic_car/scripts
cd robotic_car/scripts

# Create the node scripts and manually copy the code to each one of them
touch command_node.py
touch drive_node.py
touch opencv_node.py

# Alternatively, copy/replace the same files directly from the
# 'catkin_ws/src/robotic_car/scripts' directory in the Source_code folder provided for this article

# Make the scripts executable
chmod +x command_node.py
chmod +x drive_node.py
chmod +x opencv_node.py

# Compile the ROS package
cd ~/catkin_ws
catkin_make

# Source the workspace
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
