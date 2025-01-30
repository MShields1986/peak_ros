# peak_ros
ROS driver for use with PEAK LTPA

# Usage Instructions

This package depends on tf2-sensor-msgs. Download the appropriate distribution using the following command:

```
sudo apt install ros-noetic-tf2-sensor-msgs
```

Use catkin build and source your workspace.

The package is called with 

```
roslaunch peak_ltpa init.launch
```

In a new terminal, call either the service take_single_measurement with stream data set to false (default) or stream_data with stream data set to true.

```
rosservice call /peak/take_single_measurement "stream_data: false"
```

After this call check the other terminal for an A-scan output

or

```
rosservice call /peak/stream_data "stream_data: true"
```

After this call check rviz for something that looks like: 

![image](https://github.com/user-attachments/assets/cbbfe197-97f3-4cfd-95d3-ffc41ab652bb)

## Warning!
Depending on your mps file the reconstruction may not be accurate currently.

TODO: Read the mps file and apply focal laws for reconstruction.
