# PX4 gz_x500_mono_cam Drone Camera with QRCODE Recognition Example


## Requirements

- PX4-Autopilot
- ROS2
- Gazebo
- Colcon
- ROS2 - Gazebo Bridge
- PyzBar

## Cloning the repository

In this workspace there is a git submodule, to clone it to your machine use:

```bash
git pull --recurse-submodules
```

## How to install ros-gz bridge

This ROS2 package is used to translate Gazebo Topics to ROS2 Topics. Install executing the following command:

```bash
sudo apt-get install ros-humble-ros-gzgarden
```

## Install Python PyzBar Module

```bash
pip install pyzbar
```

## How to Build

**First, since this package requires the vision_opencv and px4_msgs packages, you need to build and source it before building this package. To do this, go to ws_common_packages workspace and follow its build instructions. Then come back here and continue with the build step.**

To build a ros2 workspace we use the  colcon build tool ([how to install](https://github.com/Ararabots-UFMS/Drone/blob/main/Tutoriais/SETUP.md)):

```bash
source /opt/ros/humble/setup.bash
source ../ws_common_packages/install/setup.bash
cd ws_drone_camera/
colcon build
```

## How to Run

Open 3 terminal windows.

Execute the following command on one terminal window:

```bash
cd PX4-Autopilot
make px4_sitl gz_x500_mono_cam
```

Execute this command on another termnal:

```bash
ros2 run ros_gz_image image_bridge /camera
```

And run the drone control on the last terminal:

```bash
source /opt/ros/humble/setup.bash
cd ws_drone_camera/
source ../ws_common_packages/install/setup.bash
source install/setup.bash
ros2 run drone_camera_py drone_camera_py
```

Gazebo should start automatically with 2 qr codes under the drone.
Also, when running the drone_camera package a new window should open with the view of the camera on the drone.

Run "commander takeoff" on the PX4-Autopilot terminal.

The drone will takeoff and start recognizing the QRCodes, on the camera view window it will mark the currently focused QRCode and show its value, after some time its going to focus on the other one.

On the terminal its going to show the direction of the QRCode in relation to the drone position.
