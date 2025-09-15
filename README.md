# AVAI Lab

This is the repository for the course "Autonomous Vehicles And Artificial Intelligence" (AVAI).


# AVAI Autonomous Race Car

## Introduction
This repository contains the **application-level software stack** for the AVAI autonomous race car (based on the F1TENTH platform).  
It provides the launch files, perception modules, mapping, and planning components needed to run the buggy in **autonomous mode**.  
The stack can be tested in **simulation** using Gazebo or deployed on the **physical buggy** together with the base software.

_________
_________

## A small note on f1tenth repo (Base layer) and how our autonomous stack linked with this 

  The autonomous buggy requires **two software layers** that must run together:

  - **Base repo** → [f1tenth_system](https://github.com/f1tenth/f1tenth_system/tree/humble-devel)  
  - **Application repo** → [avai-autonomous-race-car](https://github.com/isselab/avai-autonomous-race-car)  

---

  ### Base Layer (f1tenth_system)
  - Provides **drivers** for hardware (motors, LiDAR, IMU, camera).  
  - Publishes core topics:  
  - `/scan` (LiDAR scans)  
  - `/odom` (odometry)  
  - `/cmd_vel` (velocity commands)  
  - `/tf` (transforms)  
  - In **simulation**, Gazebo **plugins act as drivers** and publish the same topics.

---

  ### Application Layer (avai-autonomous-race-car)
  - Provides the **autonomous stack**: perception, mapping, planning, and control.  
  - Subscribes to base layer topics (`/scan`, `/odom`, `/tf`).  
  - Runs algorithms (YOLO detection, SLAM, planners).  
  - Sends driving commands back via `/cmd_vel`.

---

  ### Simulation vs. Physical Buggy
  - Thanks to ROS, **the same application code runs in both simulation and on the physical buggy**.  
  - In simulation -> drivers are replaced by Gazebo plugins.  
  - On the buggy -> drivers from the base repo handle the real sensors/actuators.  
  - Transition is usually smooth, but **driver issues may appear** on physical hardware.

---

**In short:**  
- The **base layer handles hardware communication**.  
- The **application layer makes the buggy autonomous**.  
- ROS ensures the same interfaces, so simulation and deployment can share the **same code**.

_________
_________


## Structure - Installation and Setup

The repository contains:
- folders for ROS2 packages (`_ws` - suffix, e.g. `race_car_ws`)
- the `avai_lab` python package

---

## Developer Installation

In order to use this repository you will need [ROS2 Humble](https://docs.ros.org/en/humble/index.html) (ships with python3.10).  
Ignition Gazebo (ros-gz) is used for simulation.

1. First make sure you have the latest installation tools:

```bash
python3 -m pip install --upgrade pip
pip install --upgrade packaging setuptools wheel
```

2. Install ROS dependencies:

```sh
cd race_car_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Install the `avai_lab` package as editable:

```sh
pip install -e avai_lab
```

4. Discover and install Python node dependencies (e.g. numpy, ultralytics, opencv):

```sh
cd race_car_ws/src
pip install pipreqs
pipreqs . --force --encoding=utf-8
pip install -r requirements.txt
```

5. Now go and build the ROS2 workspace and export the GAZEBO model path through IGNITION_RESOURCE:

```sh
cd race_car_ws
colcon build
export IGN_GAZEBO_RESOURCE_PATH=:/home/...YOUR WORKSPACE PATH .../src/gazebo/gazebo_f110/model (if in case your model's not loading)
source install/setup.bash
```

---

## Run (Simulation)

To start the simulation with Ignition Gazebo: using 2 launch files which are responsible for different set of nodes

```sh
ros2 launch gazebo_f110 gazebo.launch.py world:=circle
ros2 launch f110_car f110.launch.py
```

__________
__________


## Credits

This repository is based on work by:
- ossmos  
- Kevin Losing  
- Ahmad Al Shihabi  
- aloe-lia  
