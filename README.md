## Prerequisites

1. **OS**: Ubuntu 20.04
2. **ROS version** : ROS Noetic Ninjemys
3. **Git**: For cloning

## Installation & Setup

### Step 1:Install tools
```bash
sudo apt-get update
sudo apt-get install ros-noetic-desktop-full \
                     ros-noetic-navigation \
                     ros-noetic-gmapping \
                     ros-noetic-teleop-twist-keyboard \
                     python3-catkin-tools
```

### Step 2：Clone the workspace
```bash
git clone https://github.com/flyflyk/auto_watering_ws.git
```
### Step 3：Source and Build

In auto_watering_ws:
```bash
catkin build
source devel/setup.bash
``` 

## Usage

```bash
roslaunch smart_robot main.launch