# AVAI Lab

This is the repository for the course "Autonomous Vehicles And Artificial Intelligence" (AVAI).

## Structure

The respository contains:
- folders for ROS2 packages (`_ws` - suffix)
- the `avai_lab` python package

## Developer Installation

In order to use this repository you will need [ROS2 Humble](https://docs.ros.org/en/humble/index.html) (ships with python3.10).

First make sure you have the latest and greatest installation tools:

```sh
python3 -m pip install --upgrade pip
pip install --upgrade packaging setuptools
```

Install the `avai_lab` package as editable.

```sh
pip install -e avai_lab
```

Now go and build the ROS2 `test_package`.

```sh
cd race_car_ws
colcon build
source install/setup.bash
```
