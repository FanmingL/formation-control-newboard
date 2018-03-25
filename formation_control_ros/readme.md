# Formation Control ROS Package
## Get Started

### Software Requirements

- ROS kinetic (Ubuntu 16.04)
- OpenCV 3.0.0 or higher

### Install dependencies for Ubuntu 16.04 kinetic

**Install ROS**

Please follow [the installing and configuring ROS environment tutorial](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) on ROS Wiki.

**Install OpenCV**

Please follow [the installing and configuring OpenCV tutorial](https://blog.csdn.net/u013066730/article/details/79411767).


### Install Thirdparty Library

```shell
sudo apt-get update
sudo apt-get install libv4l-dev
cd driver/libapriltag
make 
sudo make install
cd ../libcam
make
sudo make install
```
### Install Thirdparty Library
Please copy the ros package to your workspace and use `catkin_make` to compile

### Function of every node
**1.uav_driver_serial**


To continuously read/write the serial port.


**2.uav_localization**


Get UAV position in real world by recognition of [AprilTags](https://april.eecs.umich.edu/software/apriltag.html).


**3.uav_proto_unpack**


Unpack the messages gotten from UAV.


**4.uav_proto_pack**


Pack the messages sent to UAV.


**5.uav_position_control**


Send control message according to the current position and target position.


**6.formation_control**


Send some simple messages to all UAVs at the same time.


**7.formation_trajectory**


Localization planner.


All of above node should be compiled on PC and onboard-PC. <br>
These nodes should be run in the onboard-PC
```shell
uav_driver_serial
uav_localization
uav_proto_unpack
uav_proto_pack
uav_position_control
```
You can launch `uav_position_control/launch/arpilot.launch` to run above.<br>
These nodes should be run in the PC
```shell
formation_control
formation_trajectory
```
You can launch `formation_control/launch/formation.launch` to run above.<br>

### Notes
1.for communication between UAVs, you'd better set your PC as master. For example, you can set the IP of your PC `192.168.1.200`, set the IP of your UAV `192.168.1.201`, and you can add these to `~/.bashrc` in your onboard-PC.
```shell
export ROS_MASTER_URI="http://192.168.1.200:11311"
export ROS_IP="192.168.1.201"
```
And then, you can get the topics which was published by onboard-PC on your PC.
