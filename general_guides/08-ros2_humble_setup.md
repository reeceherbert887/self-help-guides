# ROS2 Humble Setup Guide (Ubuntu 22.04)

This guide installs **ROS2 Humble Hawksbill** on Ubuntu 22.04.\
It does NOT include Gazebo or extra packages.


# 1. Update System

``` bash
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release
```

---

# 2. Add ROS2 Repository

Add ROS key:

``` bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add repository:

``` bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Update package list:

``` bash
sudo apt update
```

---

# 3. Install ROS2 Humble

``` bash
sudo apt install -y ros-humble-desktop
```

This installs:

-   ROS2 core
-   turtlesim
-   rqt tools
-   build tools
-   colcon support

---

# 4. Setup Environment

Add ROS2 to bashrc:

``` bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

Apply now:

``` bash
source ~/.bashrc
```

Verify:

``` bash
echo $ROS_DISTRO
```

Expected output:

    humble

---

# 5. Install Build Tools

``` bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

Initialize rosdep:

``` bash
sudo rosdep init
rosdep update
```
---


# 6. Set ROS Domain ID

Example:

``` bash
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
source ~/.bashrc
```

---


# 7. Create Workspace

``` bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
```

Add workspace to environment:

``` bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

# 8. Test Installation

Terminal 1:

``` bash
ros2 run turtlesim turtlesim_node
```

Terminal 2:

``` bash
ros2 run turtlesim turtle_teleop_key
```

If turtlesim opens, ROS2 is working correctly.

---

# Setup Complete

You can now clone your own ROS2 packages into:

    ~/ros2_ws/src

and build with:

``` bash
colcon build
```
