# PX4 gz_x500 Drone Control Example

## Requirements

- PX4-Autopilot
- Micro-XRCE-DDS-Agent (Communication between px4 and ros2)
- ROS2
- Gazebo
- Colcon

## How to Build

To build a ros2 workspace we use the  colcon build tool ([how to install](https://github.com/Ararabots-UFMS/Drone/blob/main/Tutoriais/SETUP.md)), executing the command:

```bash
source /opt/ros/humble/setup.bash
cd ws_drone_control/
colcon build
```

## How to Run

Open 3 terminal windows.

Execute the following command on one terminal window:

```bash
cd PX4-Autopilot
make px4_sitl gz_x500
```

Execute this command on another termnal:

```bash
MicroXRCEAgent udp4 -p 8888
```

And run the drone control on the last terminal:

```bash
source /opt/ros/humble/setup.bash
cd ws_drone_control/
source install/setup.bash
ros2 run px4_drone_control drone_control
```

Gazebo should start automatically and the Drone should takeoff and fly up 5 meters, go 5 meters north and 5 meters east. Then, after 20 seconds, land where it is.