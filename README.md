## Prerequisites

1. **OS**: Ubuntu 20.04
2. **ROS version** : ROS Noetic Ninjemys
3. **Git**: For cloning
```bash
sudo apt-get update
sudo apt-get install git
sudo apt-get install python3-catkin-tools
```
## Installation & Setup

### Step 1：Clone the workspace
```bash
git clone https://github.com/flyflyk/auto_watering_ws.git
```
### Step 2：Install and Build

In auto_watering_ws:
```bash
rosdep install --from-paths src --ignore-src -r -y
catkin build
``` 

### Step 3：Source the Workspace

```bash
echo "source ~/my_robot_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Usage

```bash
roslaunch smart_robot main.launch