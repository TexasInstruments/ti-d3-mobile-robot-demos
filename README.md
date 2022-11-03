TI-D3 Mobile Robot Demos
========================


<p align="left">
  <img src="docs/camera_radar_fusion_rviz.gif" />
</p>



<!-- ======================================================================= -->
## TI-D3 Mobile Robot

[![Alt text](https://img.youtube.com/vi/rrhvhDdtyF8/0.jpg)](https://www.youtube.com/watch?v=rrhvhDdtyF8)


<p align="left">
  <img src="docs/ti_d3_mobile_robot_2022-10-11.jpg" alt="drawing" style="width:512px;"/>
</p>

The TI-D3 mobile robot has following hardware components:

| HW Component                       | Part                                                                                                                        |
| ---------------------------------- | --------------------------------------------------------------------------------------------------------------------------- |
| Main Processor                     | [TDA4VM](https://www.ti.com/product/TDA4VM)                                                                                 |
| Main Processor Board               | [SK-TDA4VM](https://www.ti.com/tool/SK-TDA4VM)                                                                              |
| FPD-Link Interface Board           | [TDA4 Fusion1 Rev C board](https://svtronics.com/portfolio/evm577pfusion-v1-0-fusion/)                                      |
| Camera and Radar Fusion Sensor Kit | [FS-6843AOP-IMX390](https://www.d3engineering.com/product/designcore-fs-6843aop-imx390-fusion-sensor/)                      |
| Motor Controller                   | [TMS320F280039C](https://www.ti.com/product/TMS320F280039C), [BOOSTXL-DRV8323RS](https://www.ti.com/tool/BOOSTXL-DRV8323RS) |
| DLP Projector                      | [DLP3021LEQ1EVM](https://www.ti.com/tool/DLP3021LEQ1EVM)                                                                    |


<!-- ======================================================================= -->
## Getting Started: Software Setup

Software dependency:
- Processor SDK Linux for Edge AI 8.4
- Robotics SDK 8.4
### Preparing SD Card Image
Download the prebuilt SD card image from [this link](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/data/ti-processor-sdk-linux-sk-tda4vm-etcher-image-8.4-tid3.zip). Please note that this pre-built SD card has a few post-release patches applied on top of the Processor SDK Linux for Edge AI 8.4:
- V4L2 fix for IMX390 camera
- IWR6843AOP radar driver

Refer to the following section of Edge AI documentation: ["Preparing SD card image"](https://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-sk-tda4vm/latest/exports/docs/getting_started.html#preparing-sd-card-image).

### Robotics SDK
The TI-D3 mobile robot demos requires Robotics SDK 8.4. Referring to the following sections of the SDK documentation, please build the Robotics SDK ROS1 Docker image, and build the ROS packages on the TDA4 and on the Ubuntu PC.
- [Setting Up Robotics SDK](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/08_04_00/docs/source/docker/README.html#setting-up-robotics-sdk)
- [Docker Setup for ROS 1](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/08_04_00/docs/source/docker/setting_docker_ros1.html#docker-setup-for-ros-1)

### Build the ROS Packages
### TI-D3 Mobile Robot: Project Folder

On the SSH terminal for the TDA4, download and run `init_setup.sh`:

```
cd
wget -O init_setup.sh https://raw.githubusercontent.com/TexasInstruments/ti-d3-mobile-robot-demos/master/init_setup.sh
source ./init_setup.sh
```
The setup script performs:
- Git-clone the project git repository
- Install the mmWave radar driver ROS node
- Install ROS nodes from D3 Engineering's  git repositories
- Update DTBO overlay to use IMX390 cameras
- Install LDC binary files for IMX390 cameras

The folder structure for the TI-D3 mobile robot project is as follows:
```
/opt/ti-d3-mobile-robot-demos/ # Project folder
+ README.md
+ docs/
+ docker/
+ scripts/
+ nodes/    # catkin_make --source <this>
	+ ti_detect_go/
	+ ti_dlp_command/
	+ radar_driver
		+ ti_mmwave_tracker/
		+ serial/
	+ d3_nodes
		+ d3_fusion/
		+ d3_gamepad/
		+ d3_motorctl/
```

### Set Up Docker Environment for the Project
Build the Docker container for the project:
```
/opt/ti-d3-mobile-robot-demos/docker/docker_build.sh
```

You can check if the Docker image is successfully built:

```root@tda4vm-sk:/opt# docker images
REPOSITORY         TAG           IMAGE ID            CREATED            SIZE
j7-ros-noetic      8.4_tid3      249c4a3befa9        1 days ago         3.4GB
j7-ros-noetic      8.4           43dcdd403859        1 days ago         3.39GB
ubuntu             20.04         08f5c0d9d654        2 months ago       65.5MB
```

Run the Docker container for the project:
```
/opt/ti-d3-mobile-robot-demos/docker/docker_run.sh
```

Please note that we will use a separate ROS workspace, `$HOME/j7ros_home/tid3_ws`, for the project. We assume that Robotics SDK has been already built under `$HOME/j7ros_home/ros_ws`.
```
$HOME/j7ros_home/
+ ros_ws  # Exiting Robotic SDK workspace. Assume already ROS apps built following the SDK doc
+ tid3_ws # TI-D3 Mobile Robot project workspace
```

Build in the ROS packages inside the project Docker container:
```
cd ~/j7ros_home/tid3_ws
# This is important to use the packages from Robotics SDK
source ~/j7ros_home/ros_ws/devel/setup.bash
catkin_make --source /opt/ti-d3-mobile-robot-demos/nodes
source devel/setup.bash
```

<!-- ====================================================================== -->
### Preparing Software on Ubuntu PC

```
wget -O init_set.sh https://raw.githubusercontent.com/TexasInstruments/ti-d3-mobile-robot-demos/master/init_setup.sh
source ./init_setup.sh
```

The same folder structure, but installed under `$HOME/j7ros_home/tid3_ws/src` folder

In a similar way as in TDA4, we have two ROS Workspaces:
```
$HOME/j7ros_home/
+ ros_ws  # Exiting Robotic SDK workspace. Assume already ROS apps built follows SDK doc
+ tid3_ws # TI-D3 Mobile Robot workspace
```

Update network settings (`J7_IP_ADDR` and `PC_IP_ADDR`) in `$HOME/j7ros_home/setup_env_pc.sh`.
```
source $HOME/j7ros_home/setup_env_pc.sh
```

Run the project Docker image:
```
$HOME/j7ros_home/tid3_ws/src/ti-d3-mobile-robot-demos/docker/docker_run_pc.sh
```

Build in the ROS packages inside the project Docker container:
```
cd ~/j7ros_home/tid3_ws
# This is important to use the packages from Robotics SDK
source ~/j7ros_home/ros_ws/devel/setup.bash
catkin_make
source devel/setup.bash
```

<!-- ====================================================================== -->
### Run the Demo

**TDA4**:
Run the Docker container for the project:
```
/opt/ti-d3-mobile-robot-demos/docker/docker_run.sh
```

In the project Docker container:
```
source devel/setup.bash
roslaunch tid3_robot_demos electronica.launch
```

**PC**:
Update network settings (`J7_IP_ADDR` and `PC_IP_ADDR`) in `$HOME/j7ros_home/setup_env_pc.sh`.
```
source $HOME/j7ros_home/setup_env_pc.sh
```

Run the project Docker image:
```
$HOME/j7ros_home/tid3_ws/src/ti-d3-mobile-robot-demos/docker/docker_run_pc.sh
```

In the project Docker container:
```
roslaunch d3_fusion fusion_viz.launch
```

