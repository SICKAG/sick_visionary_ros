# sick_visionary_ros

## Overview

This repository contains the ROS drivers for Sick Visionary-T Mini CX (V3S105<area>-1AAAAAA) and Sick Visionary-S CX (V3S102<area>-1AAAAAA and V3S102<area>-1AABAAB).

### License

The source code is released under [The Unlicense](/LICENSE).

### Supported environments

The _sick_visonary_ros_ package has been tested under [ROS] Noetic 64bit on Ubuntu 20.04.

## Table of Contents

- [Supported Hardware](#supported-hardware)
- [Getting Started](#getting-started)
- [Launch Files](#launch-files)
- [Nodes](#nodes)
- [Support](#support)

## Supported Hardware

| **device name**                                              | **part no.**                                                                                                                                                                                                                                                                              | **description**                                       | **version** |
| ------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------- | :---------: |
| Visionary-T Mini CX (V3S105<area>-1AAAAAA)                   | [1112649](https://www.sick.com/de/en/catalog/products/machine-vision-and-identification/machine-vision/visionary-t-mini/v3s105-1aaaaaa/p/p665983)                                                                                                                                         | 3D structured light stereovision camera with RGB data |   ✔ 2.0.0   |
| Visionary-S CX (V3S102<area>-1AAAAAA, V3S102<area>-1AABAAB)) | [1090184](https://www.sick.com/de/en/catalog/products/machine-vision-and-identification/machine-vision/visionary-s/v3s102-1aaaaaa/p/p602149) [1090185](https://www.sick.com/de/en/catalog/products/machine-vision-and-identification/machine-vision/visionary-s/v3s102-1aabaab/p/p651629) | 3D time-of-flight camera                              |   ✔ 6.0.0   |

## Getting Started

### Installation

**1.** First you need a working **ROS installation**. [(ROS wiki)](http://wiki.ros.org/ROS/Installation)
(If you already have a running ROS environment skip to step 2.

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' # Setup your sources.list
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - #Set up your keys
sudo apt update #Update your Debian package index
sudo apt install ros-noetic-desktop ros-noetic-pcl-ros ros-noetic-pcl-conversions # install the packages
```

**2.** **Clone** this repository into your catkin workspace. Change `<YOUR_WORKSPACE>` to your actual workspace name and `<REPO_URL>` to the url provided by the Clone button.

```bash
cd ~/<YOUR_WORKSPACE>/src
git clone <REPO_URL>
```

After cloning this repository into the src folder of your catkin workspace run these additional git commands to have the correct submodules in place:

```bash
git submodule init
git submodule update
```

> **Note**
> If you've received a distribution archive just extract it into the src folder of your catkin workspace.

> **Important**
> Ensure the ROS environment is active.

```bash
source /opt/ros/noetic/setup.bash
```

**3.** **Compile and install**

```bash
cd ~/<YOUR_WORKSPACE> # move to the top level of your catkin workspace
catkin_make # Compile the drivers
catkin_make install # local install
source install/setup.bash # source your workspace on top of your environment
```

### Quick Start

> **Important**
> Ensure the camera device is connected to your computer.
>
> 1. Connect your device via Ethernet to your local PC
> 2. Connect the device to the power supply and wait until it has booted up

**1.** Use the launch file to start the node. For the Visionary-T Mini CX run:

```bash
roslaunch sick_visionary_ros sick_visionary-t_mini.launch
```

> **Note**
> Please use the device specific launch file to start the correct driver variant.

**2.** Visualize the results in RViz (3D visualization tool for ROS).
Open a new terminal and run

```bash
rosrun rviz rviz
```

You should see the rviz window pop up.
In the _Displays_ window (inside Rviz) under _Global Options_, set _Fixed Frame_ to `camera` to match the default frame*id (see [parameters](#parameters)).
Subscibre to the topic you want to be displayed. \*\*For example to view the \_Depth map*\*\*:
_Add > By Topic > /sick_visionary_t_mini > /depth > Camera > Ok_
You should see a new display in the main Rviz window which visualises the topic data.

> **Note**
> Optionally save the Rviz configuration to skip setting the parameters and windows at the next restart. Simply close the Rviz window and follow the instructions.

## Launch files

There is one launch file for each supported device.

- `sick_visionary-s.launch`
- `sick_visionary-t_mini.launch`

The launch file starts the respective ROS Node and provides it with the necessary parameters and arguments.

> **Note**
> The driver connects to the default IP and API-port (2114) of the device. To adapt this for your needs just edit the launch file for the specific variant.

### Arguments

- `camera:`
  Name of the device. (default: sick_visionary_t_mini or sick_visionary_s)

### Parameters

#### sick_visionary-s.launch

- `remote_device_ip:`
  IP-address of the device (string, default: 192.168.1.10)

- `frame_id:`
  Name of the reference frame. (string, default: camera)

- `enable_z:`
  Enables publishing of z-topic. (bool, default: true)

- `enable_statemap:`
  Enables publishing of statemap-topic. (bool, default: true)

- `enable_rgba:`
  Enables publishing of rgba-topic. (bool, default: true)

- `enable_points:`
  Enables publishing of points-topic. (bool, default: true)

- `desired_frequency:`
  Sets the desired frequency in the diagnostics-topic.
  Min and max frequencies are derived from it (double, default: 15.0)

#### sick_visionary-t_mini.launch

- `remote_device_ip:`
  IP-address of the device (string, default: 192.168.1.10)

- `frame_id:`
  Name of the reference frame. (string, default: camera)

- `enable_depth:`
  Enables publishing of depth-topic. (bool, default: true)

- `enable_statemap:`
  Enables publishing of statemap-topic. (bool, default: true)

- `enable_intensity:`
  Enables publishing of intensity-topic. (bool, default: true)

- `enable_points:`
  Enables publishing of points-topic. (bool, default: true)

- `desired_frequency:`
  Sets the desired frequency in the diagnostics-topic.
  Min and max frequencies are derived from it (double, default: 15.0)

## Nodes

### sick_visionary_s_node

Visionary-S is a 3D camera based on structured light stereovision. It provides real-time 3D and RGB data at up to 30 frame per second (fps).

#### Published Topics:

- `/camera_info` [(sensor_msgs/CameraInfo)](https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html) Camera calibration and metadata.
- `/z` [(sensor_msgs/Image)](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html) Depicts distance to camera. Contains uint16 distance values.
- `/points` [(sensor_msgs/PointCloud2)](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) Contains XYZ point cloud [m]
- `/statemap` [(sensor_msgs/Image)](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html) Contains uint16 statemap values which help to understand wheter the data is missing due to a configured filter or due to other circumstances e.g. saturation effects.
- `/rgba` [(sensor_msgs/Image)](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html) RGBA color image.

### sick_visionary_t_mini

Visionary-T Mini is a 3D camera based on the time-of-flight (TOF) principle. It provides real-time 3D and 2D data at up to 30 frame per second (fps).

#### Published Topics:

- `/camera_info` [(sensor_msgs/CameraInfo)](https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html) Camera calibration and metadata.
- `/depth` [(sensor_msgs/Image)](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html) Contains uint16 radial distance values [1/4 mm]
- `/points` [(sensor_msgs/PointCloud2)](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) Contains XYZ point cloud [m]
- `/intensity` [(sensor_msgs/Image)](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html) Raw image from device. Contains uint16 intensity values. Helpful for visual identification of the scene.
- `/statemap` [(sensor_msgs/Image)](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html) Contains uint16 statemap values which help to understand wheter the data is missing due to a configured filter or due to other circumstances e.g. saturation effects.

## Support

Depending on the nature of your question, there are two support channels:

1. For questions regarding the code shared in this repo please check the FAQ first and [search if an issue already exists](../../issues). If a related issue doesn't exist, you can open a new issue using the [issue form](../../issues/new/choose).
2. For application or device specific questions look for common solutions and knowledge articles on the [Sick Support Portal](https://support.sick.com/). If your question is not answered there, open a ticket on the [Sick Support Portal](https://support.sick.com/).

## Keywords

Visionary-S
Visionary-T Mini
ROS
SICK
CX
