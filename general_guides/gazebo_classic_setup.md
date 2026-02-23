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
