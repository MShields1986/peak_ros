# peak_ros
ROS driver for use with PEAK LTPA

## Installation

### Docker
```bash
git clone https://github.com/MShields1986/peak_ros.git
cd peak_ros
./run.sh
```

### Host System
This package depends on tf2-sensor-msgs. Download the appropriate distribution using the following command:

```bash
sudo apt update
sudo apt install ros-noetic-tf2-sensor-msgs
```

```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/MShields1986/peak_ros.git
```

Use catkin build and source your workspace.

## Usage

```bash
roslaunch peak_ltpa init.launch
```

Call either of the services `/peak/take_single_measurement` or `/peak/stream_data`.

```bash
rosservice call /peak/take_single_measurement "take_single_measurement: true"
```

...or...

```bash
rosservice call /peak/stream_data "stream_data: true"
```

After this RViz ought to show the current b scan as a pointcloud. 

![image](https://github.com/user-attachments/assets/cbbfe197-97f3-4cfd-95d3-ffc41ab652bb)

### Warning!
Depending on your mps file the reconstruction may not be accurate currently.

TODO: Read the mps file and apply focal laws for reconstruction.

## Bugs and Feature Requests
Please report bugs and request features using the [Issue Tracker](https://github.com/MShields1986/peak_ros/issues).
