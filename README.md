## Prerequisites

1. **OS**: Ubuntu 20.04
2. **ROS version** : ROS Noetic Ninjemys
3. **Git**: For cloning

## Installation & Setup

### Step 1:Install tools
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-noetic-navigation ros-noetic-map-server ros-noetic-amcl
```

### Step 2：Clone the workspace
```bash
git clone https://github.com/flyflyk/auto_watering_ws.git
```
### Step 3：Source and Build

In auto_watering_ws:
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/auto_watering_ws/devel/setup.bash" >> ~/.bashrc # Correct the path
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/auto_watering_ws/src/smart_robot/models' >> ~/.bashrc # Correct the path
source ~/.bashrc
catkin build
``` 

## Usage

To launch:
```bash
roslaunch smart_robot main.launch

To Stop:
Ctrl + C in the console the exit.