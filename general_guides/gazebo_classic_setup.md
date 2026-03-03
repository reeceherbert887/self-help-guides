# Gazebo Classic Setup Guide (Ubuntu 22.04 + ROS2 Humble)

This guide installs **Gazebo Classic (Gazebo 11)** and integrates it
with **ROS2 Humble**.

This does NOT install TurtleBot or any simulation packages.


# 1. Install Gazebo Classic

Update package list:

``` bash
sudo apt update
```

Install Gazebo and development libraries:

``` bash
sudo apt install -y gazebo libgazebo-dev
```

Verify installation:

``` bash
gazebo --version
```

Expected output example:

    Gazebo multi-robot simulator, version 11.x.x



# 2. Install ROS2 Gazebo Integration

Install ROS2 Gazebo bridge packages:

``` bash
sudo apt install -y ros-humble-gazebo-ros-pkgs
```

Install ROS2 control integration:

``` bash
sudo apt install -y ros-humble-gazebo-ros2-control
```


# 3. Configure Gazebo Environment

Source Gazebo setup script:

``` bash
source /usr/share/gazebo/setup.sh
```

Add permanently:

``` bash
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
source ~/.bashrc
```



# 4. Verify ROS2 Integration

Check Gazebo ROS packages:

``` bash
ros2 pkg list | grep gazebo
```

Expected packages include:

-   gazebo_ros
-   gazebo_plugins
-   gazebo_msgs
-   gazebo_ros_pkgs
-   gazebo_ros2_control



# 5. Test Gazebo

Launch Gazebo:

``` bash
gazebo
```

Close Gazebo properly using:

    Ctrl + C



# Setup Complete

Gazebo Classic is now installed and ready for ROS2 integration.

You can now clone simulation packages into your ROS2 workspace:

    ~/workspace_name/src/

and build using:

``` bash
colcon build
```

Running Turtlebot3 Demo in Gazebo
[For your personal PC with admin rights]
Please don’t try it on the Lab PCs

The objective of this task is to get you familiar with the overall picture (ROS + Gazebo
environment) inspired by Turtlebot 3.

Since, we will be using TurtleBot 3, we need to first install “TurtleBot3 Simulator”. Here are the
instructions to load software packages for the TurtleBot simulator.

First let us install ‘git’ command in our Ubuntu (if it is not already there):
```
sudo apt install git
```

Before we install the TurtleBot3 Simulation, we need to make sure that the prerequisite
packages turtlebot3 and turtlebot3 are there. Without these packages, the simulation cannot be
launched. So, first let us install these dependent packages.

Enter the following commands, one after the other:
```
cd ~/ros2_ws/src/
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```


Note: DynamixelSDK is a software development kit that provides control functions using packet
communication (Required to work with real robot).
Although the package ‘ros-humble-gazebo-ros-pkgs’ is pre-installed on the Robotics Lab. PCs, but just
in case, if you need to install in your personal Ubuntu machines:
```
sudo apt install ros-humble-gazebo-ros-pkgs
```

If, for some reason, this does not work, try:

```
sudo apt update
sudo apt install --reinstall gazebo ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```

We need to build,

```
cd ~/ros2_ws && colcon build
```
If not done already, add the following line in your bashrc file to avoid sourcing again and again.
```
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

Now that we installed the pre-requisite packages, let us install TurtleBot3 Simulation Package
```
cd ~/ros2_ws/src/
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/ros2_ws && colcon build
```

To view the turtleBot3 packages downloaded in each category, type the following command:
```
ros2 pkg list | grep turtlebot3
```

TurtleBot3 has three models: Burger, Waffle, and Waffle Pi, so we need to set the model before
launching TurtleBot3. Type this command to open the bashrc file:
```
sudo gedit ~/.bashrc
```

Add this line at the bottom of the file:
```
export TURTLEBOT3_MODEL=burger
```

Save the file, close it and then source it.
```
source ~/.bashrc
```