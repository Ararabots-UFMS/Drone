# Common Packages 

The packages in this workspace are used in one or more example workspace, so they don't need to be copied everywhere

## Requirements

- ROS2
- Colcon

## How to Build

To build a ros2 workspace we use the  colcon build tool ([how to install](https://github.com/Ararabots-UFMS/Drone/blob/main/Tutoriais/SETUP.md)):

Change {num_workers} for how many concurrently workers (CPU Cores) you want to use to build the packages. Recommend using more than 1 because it takes some time to build these packages. 

```bash
    source /opt/ros/humble/setup.bash
    cd ws_common_packages/
    colcon build --parallel-workers {num_workers}
```
