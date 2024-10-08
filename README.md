<div align="center">

# Mobilebot
Autonomous Navigation of a 2-wheeled Differential drive mobile robot using ROS and Gazebo.

![Static Badge](https://img.shields.io/badge/Ubuntu-20.04_(Focal_Fossa)-orange)
![Static Badge](https://img.shields.io/badge/ROS-Noetic_Ninjemys-lightgreen)
![Static Badge](https://img.shields.io/badge/Python-3.8.10-red)
![Static Badge](https://img.shields.io/badge/Gazebo-11+-blue)
![Static Badge](https://img.shields.io/badge/Mobile_Robot-Diff_Drive_(2_Wheeled)-yellow)
![Static Badge](https://img.shields.io/badge/LICENSE-MIT-violet)

<br/>

![image](https://github.com/VibhuSharma19/Mobilebot_Navigation/blob/master/images/mobilebot.png)
</div>

## Table of Contents
* [Overview](#overview)<br/>
  - [Features](#features)
* [Getting Started](#getting-stated)<br/>
  - [Prerequists](#prerequists)
  - [Installation and Build](#installation-and-build)
* [Usage](#usage)
  -  [Gazebo Simulation](#gazebo-simulation)
  -  [SLAM](#slam)
  -  [Autonomous Navigation](#autonomous-navigation)
* [Project Visuals](#project-visuals)
* [Contributing](#contributing)
* [Contact](#contact)
* [License](#license)
* [Acknowledgements](#acknowledgements)




## Overview
__Mobilebot__ is a 2-wheeled _Differential Drive_ autonomous robot simulation project, designed for navigation, exploring and research. This project provides various features and cutting-edge technologies intergartion. Mobilebot's architechture combines sensor intergration, localization, SLAM and navigation to enable seamless autonomous operation. The project aims to demostrate the potential of autonomous robot in various applications and through various custom solutions and features.

### Features
* __Dual Controller__: Self-coded controller and Differential drive controller for precise motion control.
* __Sensor Integration__: Currently intergrates three sensors
  - 360 deg LASER Scanner for environment mapping and navigation
  - Camera (RGB) Sensor for visual feedback 
  - IMU sensor for orientation and localization
* __ROS Bag__: Record and replay the sensors output using ROS Bag feature. The recording can be used for ML algoritm training, Cartogarpher SLAM method and many more applications.
* __Web Video Stream__: The camera sensor output can be seen in the westream and used in cloud services and as required for other applications.
* __Localization__: Kalman filter with sensor fusion and Extended kalman filter based localization with custom-implemented sensors deviations.
* __SLAM__: Currently able to use Gmapping and Hector algorithm for unknown environment mapping. Choose from multiple _Odometry_ publishers like Differential drive plugin, self-coded controller, and kalman filter implemented publishing.
* __Bot Commanding__: The bot motion can be commanded through three different ways
  - Commanding using controller and _/cmd_vel_ topic
  - Using teleop_twist_keyboard interface (replace _cmd_vel_ to mobilebot's _cmd_vel_ topic)
  - Using Joystick controller
* __Autonomous Navigation__: Navigation goals can be provided using two different ways
  - Using Rviz interactive target provider
  - Using annotated map location  
  
## Getting Stated
The overall project is divided into two sub-projects. Main project repository as [Mobilebot](https://github.com/VibhuSharma19/Mobilebot.git) contains the bot description, gazebo simulation and kalman filter implementation. The secondary (required) sub-project repository [Mobilebot_Navigation](https://github.com/VibhuSharma19/Mobilebot_Navigation.git) focuses on SLAM and Navigation implementaion for the Mobilebot. Both the repositories are required to use all the features of the project.

### Prerequists
* __Ubuntu__: 20.04 (_Focal Fossa_)
* __Robot Operating System (ROS)__: Noetic Ninjems
* __Gazebo__: 11 or higher (_Classic_)
* __Python__: 3.8 or higher
* __ROS Navigation Stack__

### Installation and Build
1. Clone and build the whole project (both the sub-projects):
```sh
$ cd ~/catkin_ws/src  
$ git clone --recurse-submodules https://github.com/VibhuSharma19/Mobilebot.git
$ cd ~/catkin_ws && rosdep install -r --ignore-src --from-paths src 
$ catkin_make
```

1. Clone and build the repository separately for Gazebo and Navigation package:
```sh
$ cd ~/catkin_ws/src  
$ git clone https://github.com/VibhuSharma19/Mobilebot.git  
$ git clone https://github.com/VibhuSharma19/Mobilebot_Navigation.git
$ cd ~/catkin_ws && rosdep install -r --ignore-src --from-paths src 
$ catkin_make
```

## Usage

### Gazebo Simulation

1. Gazebo world launch
```bash
# Empty World
$ roslaunch mobilebot gazebo.launch

# Office Environment
$ roslaunch mobilebot office.launch 
```
2. Controller launch
```bash
# Self-coded Controller
$ roslaunch mobilebot controller.launch 

# Differential-drive Controller
$ roslaunch mobilebot controller.launch is_simple:=false
```
3. Bot commanding
```bash
# Using controller command
$ rostopic pub /mobilebot_controller/cmd_vel <vel_cmd>

# Using Teleop Twist Keyboard 
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=<mobilebot cmd_vel topic>

# Using Joystick Controller
$ roslaunch mobilebot joystick_teleop.launch 
#change the '/dev' param to your controller path
```

### SLAM 

1. Launch the office or custom Gazebo world
2. Launch the controller (preferred Diff-drive controller)
3. SLAM implementaion
```bash
# Gmapping algorithm
$ roslaunch mobilebot_navigation slam.launch slam_method:=gmapping

# Hector algorithm
$ roslaunch mobilebot_navigation slam.launch slam_method:=hector
```
4. Launch bot commanding interface
5. Explore the environment via the bot and create and visualize the map in Rviz
6. Save the map
```bash
$ rosrun map_server map_saver -f ~/map
```

### Autonomous Navigation
1. Launch the Gazebo World
2. Navigation implementation
```bash 
$ roslaunch mobilebot_navigation navigation.launch
```
3. Giving navigation target
```bash
# Using Rviz
# Click on '2D Nav Goal' in Rviz and than click inside map to define target point and orientation

# Using Location Name
$ rosrun mobilebot_navigation send_goal.py
```

## Project Visuals

* Mobilebot in Gazebo World
<p align="center">
<img src="https://github.com/VibhuSharma19/Mobilebot_Navigation/blob/master/images/gazebo.jpg">
</p>

* Mobilebot in Office Environment
<p align="center">
<img src="https://github.com/VibhuSharma19/Mobilebot_Navigation/blob/master/images/Gazebo_office.jpg">
</p>

* SLAM implementation
<p align="center">
<img src="https://github.com/VibhuSharma19/Mobilebot_Navigation/blob/master/images/slam.png">
</p>

* Office Map using Gmapping
<p align="center">
<img src="https://github.com/VibhuSharma19/Mobilebot_Navigation/blob/master/images/map_gmapping.png">
</p>

* Mobilebot Tf tree
<p align="center">
<img src="https://github.com/VibhuSharma19/Mobilebot_Navigation/blob/master/images/gmapping_TFtree.png">
</p>

* Office Annotated map for Navigation
<p align="center">
<img src="https://github.com/VibhuSharma19/Mobilebot_Navigation/blob/master/images/Annotated_map.png">
</p>

* Navigation using annotated Office environment locations


https://github.com/user-attachments/assets/d8c9e302-2433-4fd5-a90a-3d8045c57293



## Contributing

Pull requests and issues are always welcome. You can add additional new features byccreating branches and commit changes. Please submit your contribution and report any bugs or issues.


1. Fork the Project
2. Create your New_Feature branch `git checkout -b feature/New_Feature`
3. Commit your changes `git commit -m 'Add some New_Feature'`
4. Push to the branch `git push origin feature/New_Feature`
5. Open a Pull Request

## Contact

__*VIBHU SHARMA*__  
_M.Tech (Gold Medalist :1st_place_medal: ) - Automation and Robotics_ <br/>
[![Static Badge](https://img.shields.io/badge/LinkedIn-Vibhu_Sharma-blue)](www.linkedin.com/in/-vibhu-sharma) 
[![Static Badge](https://img.shields.io/badge/Github-VibhuSharma19-white)](https://github.com/VibhuSharma19)

## License
[MIT LICENSE](https://github.com/VibhuSharma19/Mobilebot/blob/main/LICENSE)

## Acknowledgements

* [Self-Driving and ROS](https://www.udemy.com/course/self-driving-and-ros-learn-by-doing-odometry-control/?couponCode=KEEPLEARNING) course in __Udemy__
* [Basic of ROS](https://www.udemy.com/course/ros-navigation/?couponCode=KEEPLEARNING) course in __Udemy__

