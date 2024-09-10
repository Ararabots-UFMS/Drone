# Simulations

## Requirements

- PX4-Autopilot
- ROS2
- Gazebo
- Colcon

## Dependencies

Some of examples requires dependencies that are located in the *ws_common_packages* workspace. If the *src* folder is empty for you, run the following commands:

```bash
cd ws_common_packages/src
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/ros-perception/vision_opencv.git
```

Then, follow the instructions on [how to build](https://github.com/Ararabots-UFMS/Drone/tree/main/FlightCodes/Examples/ws_common_packages) 

## Fixes

Also, its important to change the python setuptools package version, because it won't let you build the code if its in another version:

```bash
pip install setuptools==58.2.0
```
