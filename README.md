# PX4_simulation
Drone simulation in gazebo
> **Note** This simulation environment is built in ubuntu 20.04

## Contents

* [Github Setting](#github-setting)
* [Installation](#installation)
* [Usage](#usage)
    * [Structure](#structure)
    * [Joysticks](#joysticks)
    * [Convention](#convention)
* [Function](#function)
    * [Tracking](#tracking)
    * [Gimbal Control](#gimbal-control)
    * [Obstacle Avoidance](#obstacle-avoidance)
* [Reference](#reference)

## Github setting

* being a contributor of this repository

* setting ssh key

  > **Note** please refer to [this website](https://blog.jaycetyle.com/2018/02/github-ssh/)

## Installation

* install ros noetic

  1. Add ROS to sources.list:
     
         sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
         sudo apt install curl # if you haven't already installed curl
         curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
         sudo apt update
     

  1. Install ROS with Gazebo:

         sudo apt install ros-noetic-desktop-full
         source /opt/ros/noetic/setup.bash

  1. Install and initialize rosdep.

         sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
         sudo apt install python3-rosdep
         sudo rosdep init
         rosdep update

  1. ros test

         roscore
  
  > **Note** please refer to [this website](http://wiki.ros.org/noetic/Installation/Ubuntu) to get more installation details

* install PX4_simulation package (using ssh)
      
      cd ~
      mkdir -p ~/PX4_simulation_ws/src
      cd ~/PX4_simulation/src
      git clone --recursive git@github.com:Aiseed/PX4_simulation.git
      cd ..
      catkin_make
      
* install PX4-Autopilot package (using ssh)
      
      cd ~
      git clone --recursive git@github.com:Aiseed/PX4-Autopilot.git


* install ubuntu.sh (includes all the dependency required by the simulation tools)
      
      cd ~
      bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
      
* install mavros
      
      cd ~
      sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
      
* install GeographicLib datasets
      
      cd ~
      wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
      sudo bash ./install_geographiclib_datasets.sh
      
* install obstacle avoidance dependencies
      
      sudo apt install ros-noetic-stereo-image-proc ros-noetic-image-view
      sudo apt install libpcl1 ros-noetic-octomap-*
      # Gstreamer plugins (for Gazebo camera)
      sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly libgstreamer-plugins-base1.0-dev
* download weight for object detection
  * download tiny-yolov4.weight
* install joystick dependencies

      sudo apt-get install ros-noetic-joy
      sudo apt-get install libspnav-dev libbluetooth-dev libcwiid-dev

* install .bash file (using ssh)
      
      cd ~
      git clone git@github.com:Aiseed/.bash.git

* source bash file (add environment variable)
  * open the .bashrc file
  
        cd ~
        vim ~/.bashrc
  * add the below to the bottom of .bashrc file
        
        source ~/.bash/.ros_bash
        source ~/.bash/.px4_bash
        source ~/.bash/.env_bash (for tracking)
        
* quick test (without AI, gimbal control and tracking)
  * launch with quadcopter
        
        roslaunch control px4_mavros_joy.launch
      
      > **Note** you can launch with parameter "type" to launch different type of vehicles</br>
        
        roslaunch control px4_mavros_joy.launch type:=_vtol
      
      or
      
        roslaunch control px4_mavros_joy.launch type:=_vtol_gimbal

## Usage
### Structure
![](resources/structure.png)
### Joysticks
* vehicle joy mapping
  ![](resources/vehicle_joy_mapping.png)

* gimbal joy mapping
  ![](resources/gimbal_joy_mapping.png)

### Convention
* every modified models are all located in the ***~/PX4-Autopilot/Tools/sitl_gazebo/models*** with post_prefix ***aiseed***
* every modified worlds are all located in the ***~/PX4-Autopilot/Tools/sitl_gazebo/worlds*** with post_prefix ***aiseed***
* most of the often-use launch file are located in the ***control*** package 
* launch files' name follows the rules below

       (function)_(function)_(function)_(function).launch
   
   >  Function lists
   >  
   >  mavros : ros & mavlink bridge
   >  
   >  px4 : px4 firmware simulator
   >  
   >  joy : joysticks driver
   >  
   >  ui : objection detection & user interface
   
   ex. if you want to test the function of mavros px4 and with joysticks control, you can use
   
       roslaunch control px4_mavros_joy.launch
       
   if you want to add the function of object detection
       
       roslaunch control ui_px4_mavros_joy.launch
   

## Function
### Tracking
> **Note** this function is specific to multiroter
>
> **Note** you need to turn on **ui** function
>

ex. 

    roslaunch control ui_px4_mavros_joy.launch
    
and open the second terminal, run

    rosrun commander commander_offboard.py
    
you can fly to aiseed person, and ensure the detection result showing on the ui. Then you can switch to offboard mode, the drone should start to follow the aiseed person(you can choose the following target by modifying ***commander_offboard.py***)

> **Note** feel free to implement any offboard mode function in the ***commander_offboard.py***. ex. tracking, path planning, obstacle avoidance ...etc.  

### Gimbal Control
> **Note** you need two xbox joysticks to control vehicle and gimbal at the same time
> 
> **Note** this function is specific to using *type:=_vtol_gimbal*

ex. 

    roslaunch control px4_mavros_joy.launch type:=_vtol_gimbal
    
and open the second terminal, run

    rosrun gimbal_control xbox_gimbal.py

### Obstacle Avoidance
> **Note** we only use *local_planner*
> 
> **Note** you can change **obstacle_cost_param_** which represents the distance between obstacle and the drone that will dominate the cost function by running `rosrun rqt_reconfigure rqt_reconfigure` 

* simulation

       roslaunch local_planner local_planner_depth-camera.launch
    
> **Note** you can turn on gazebo client by modifying *argument gui* into *true* which is located in the ***avoidance_sitl_mavros.launch***

* hardware


> **Note** please refer to [this website](https://github.com/Aiseed/PX4-Avoidance) to get more OA detailed information
## Reference
