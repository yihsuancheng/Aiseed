# PX4_simulation
drone simulation in gazebo
> **Note** This simulation environment is built in ubuntu 20.04

## Contents

* [Github Setting](#github-setting)
* [Installation](#installation)
* [Usage](#usage)
    * [Tracking](#tracking)
    * [Obstacle Avoidance](#obstacle-avoidance)
    * [Gimbal Control](#gimbal-control)
* [Reference](#reference)

## Github setting

* add to contributor

* setting ssh key

## Installation

* install ros noetic

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



## Reference
