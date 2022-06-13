# PX4_simulation
drone simulation in gazebo

## Contents

## Github setting

* add to contributor

* setting ssh key

## Installation

* install PX4_simulation package (using ssh)
      
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

* install .bash file (using ssh)
      
      cd ~
      git clone git@github.com:Aiseed/.bash.git

* source bash file (add environment variable)

      cd ~
      vim ~/.bashrc
      
* test




## Reference
