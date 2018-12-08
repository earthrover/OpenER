INSTALL
===============
Install the core of the ROS and rest of the packages can be added manually
```
sudo apt-get install ros-kinetic-ros-base
```
Run roscore to confirm ROS installation. Install Gazebo 9 using commands:

The  standard ROS kinetic install includes Gazebo 7 (ros-kinetic-desktop-full) so we have to install Gazebo 9 separately
```
sudo apt-get install ros-kinetic-gazebo9-ros-pkgs ros-kinetic-gazebo9-ros-control ros-kinetic-gazebo9*
```

#Run gazeboto confirm Gazebo installation.

 

#Adding other ROS packages and dependencies
```
sudo apt-get -y install ros-kinetic-catkin
sudo apt-get -y install rviz
sudo apt-get -y install ros-kinetic-gazebo9-ros ros-kinetic-kdl-conversions 
sudo apt-get -y install ros-kinetic-controller-manager ros-kinetic-joint-state-controller ros-kinetic-controller-interface ros-kinetic-four-wheel-steering-msgs 
sudo apt-get -y install ros-kinetic-joint-trajectory-controller ros-kinetic-rqt ros-kinetic-rqt-controller-manager 
sudo apt-get -y install ros-kinetic-rqt-joint-trajectory-controller ros-kinetic-ros-control ros-kinetic-rqt-gui
sudo apt-get -y install ros-kinetic-rqt-plot ros-kinetic-rqt-graph ros-kinetic-rqt-rviz ros-kinetic-rqt-tf-tree
sudo apt-get -y install ros-kinetic-kdl-parser ros-kinetic-forward-command-controller ros-kinetic-tf-conversions 
sudo apt-get -y install ros-kinetic-xacro ros-kinetic-joint-state-publisher ros-kinetic-robot-state-publisher
sudo apt-get -y install ros-kinetic-ros-control ros-kinetic-ros-controllers
```

###### To compile our controllers we need the controller interface
```
sudo apt-get -y install ros-kinetic-teleop-twist-joy ros-kinetic-jsk-teleop-joy ros-kinetic-joy-teleop
sudo apt-get -y install ros-kinetic-realtime-tools
sudo apt-get -y install ros-kinetic-urdf-geometry-parser

sudo apt-get -y install ros-kinetic-ros-control ros-kinetic-ros-controllers 
sudo apt-get -y install ros-kinetic-control-toolbox ros-kinetic-velocity-controllers
sudo apt-get -y install ros-kinetic-joint-state-controller ros-kinetic-effort-controllers 
sudo apt-get -y install ros-kinetic-position-controllers ros-kinetic-joy
```
# Navigation packages
```
sudo apt-get -y install ros-kinetic-navigation
sudo apt-get -y install ros-kinetic-move-base ros-kinetic-move-base-msgs ros-kinetic-teb-local-planner ros-kinetic-teb-local-planner-tutorials
sudo apt-get -y install ros-kinetic-robot-localization

