# This readme file is currently under construction

Please be aware that this readme file is a work in progress and may not be complete. We will be updating it regularly with new information, so please check back later for updates.

# Harpia - A Hybrid System for UAV Missions

## About the project

Harpia is a system for UAV mission and path planning. The project aims to provides a set of tools and libraries for generate UAV autonomous missions in a high-level for the user. 

## The Architecture

## Instalation
<aside>
💡 Make sure that the system is updated →`sudo apt-get update`
</aside>

### System Versions

- Ubuntu: Ubuntu 18 LTS
- ROS → Melodic
- QGroundControl → v4.2.1

### Dependecies

- `sudo apt install curl libc6 libstdc++6 openjdk-11-jdk python3-prettytable python3-pip python3-lxml libxml2 libxslt1.1`
- `sudo apt-get install flex bison python3-opencv python3-matplotlib libxml2 libxslt1-dev`
- `sudo apt install git`
- `sudo pip3 install git+https://github.com/catkin/catkin_tools.git`
- `sudo pip3 install git+https://github.com/colcon/colcon-common-extensions`
- `sudo -H pip3 install --upgrade pip`
- `pip install pyAgrum termcolor toml empy packaging jinja2 rospkg pandas pyproj shapely spicy scikit-learn psutil install future testresources kconfiglib jsonschema sympy==1.7.1 graphviz lxml  seaborn keras tensorflow pyspark plotly cloudpickle jupyter jupyterlab pyros-genmsg`

### Ros Installation

- ROS Installation on Ubuntu → [official link](http://wiki.ros.org/melodic/Installation/Ubuntu)
- `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
- `curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`
- `sudo apt update`
- `sudo apt install ros-melodic-desktop-full`
- `echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc`
- `source ~/.bashrc`
- `sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential`
- `sudo apt-get install python-jinja2`
- `sudo pip install numpy toml`
- `sudo apt install python-rosdep`
- `sudo rosdep init`
- `rosdep update`

### MavROS Installation:

- [MAVROS documentation](http://wiki.ros.org/mavros)
- [MAVROS installation guide](https://docs.px4.io/main/en/ros/mavros_installation.html)
- `sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras`
- `sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs`
- `wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts`
- `sudo bash ./install_geographiclib_datasets.sh`
- `sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev`

### QGroundControl Installation:
- [QGroundControl site](http://qgroundcontrol.com/)
- Download the app [here](https://github.com/mavlink/qgroundcontrol/releases/download/v4.1.6/QGroundControl.AppImage)
- `sudo usermod -a -G dialout $USER`
- `sudo apt-get remove modemmanager -y`
- `sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y`
- LogOut and Login again
- `chmod +x ./QGroundControl.AppImage`
- Run `./QGroundControl.AppImage`

### PX4 Firmware:
- [PX4 Firmware site](https://docs.px4.io/main/en/)
- `git clone --depth=1 --branch v1.13.2 https://github.com/PX4/PX4-Autopilot/`
- `cd PX4-Autopilot`
- `make`
- `bash ./Tools/setup/ubuntu.sh`


### Project setup

- `git clone https://github.com/p4aicmc/harpia_test.git`

### Things you might need to do

- In the files `/opt/ros/noetic/share/mavros/launch/apm_config.yaml`, `/opt/ros/noetic/share/mavros/launch/px4_config.yaml` and `/opt/ros/noetic/share/mavros_extras/launch/px4flow_config.yaml` change the `timesync_rate` value to `0.0`.

### Build

- `cd harpia`
- `catkin build`

- go to `harpia/src/rosplan/rosplan_planning_system` and unzip common folder

## Running the system
### Starting the drone simulation

Open QGroundControl

### Launcher
- `Check if PX4-Autopilot is on the home directory of the user, otherwise change the px4path variable in the launcher.py file`
- `python3 launcher.py`

#### Terminal 1
- `sudo su`
- `cd PX4-Autopilot`
- `export PX4_HOME_LAT=-22.001333; export PX4_HOME_LON=-47.934152; export PX4_HOME_ALT=847.142652; make px4_sitl gazebo`

#### Starting harpia system
##### Terminal 2
- `cd harpia_test`
- `source devel/setup.bash`
- `roslauch harpia.lauch`

#### Starting a new mission
##### Terminal 3
- `cd harpia_test`
- `source devel/setup.bash`
- `rosrun mission_planning teste_client.py <ID_MISSION> <ID_MAP> <ID_DRONE>`

Example: running `rosrun mission_planning teste_client.py 1 2 0` will execute the mission 1 for map 2 with drone 0. The missions, maps and drones are defined in the json files in hapria/json

## Simulation Video

[![Video](https://i9.ytimg.com/vi_webp/--hn0I5QUJ8/mq2.webp?sqp=CJzBop4G-oaymwEmCMACELQB8quKqQMa8AEB-AHUBoAC4AOKAgwIABABGGQgZShUMA8=&rs=AOn4CLALXTaHg7IRncNrzhT9RfPaIgf7Pg)](https://youtu.be/--hn0I5QUJ8)

## Articles 
- [Service-Oriented Architecture to Integrate Flight Safety and Mission Management Subsystems into UAVs, ICAS, BeloHorizonte, 2018](https://www.icas.org/ICAS_ARCHIVE/ICAS2018/data/papers/ICAS2018_0374_paper.pdf)
- [Harpia: A Hybrid System for Agricultural UAV Missions](https://authors.elsevier.com/tracking/article/details.do?surname=Vannini&aid=100191&jid=ATECH)

## Implementation Schedule 

## References

### ROSPlan

ROSPlan (Robot Operating System Planning) is a planning framework for robotic systems that uses the Planning Domain Definition Language (PDDL) to describe the problem domain. It is an open-source project hosted on GitHub, and can be found at the following repository: https://github.com/KCL-Planning/ROSPlan

This repository is maintained by the Planning group at King's College London, and provides a set of tools and libraries for integrating planning capabilities into robotic systems using the ROS (Robot Operating System) framework, used in this project (Harpia). The ROSPlan project provides a PDDL compiler for generating plans, as well as a ROS-based implementation of the PDDL forward-planning algorithm. The repository also includes example problem domains and sample code for using the system with various robotic platforms.

ROSPlan is widely used in the field of automated planning for robotics, it has been used in multiple research projects and also in industry. It is actively developed and maintained by the research group at King's College London and has a growing community of contributors.

