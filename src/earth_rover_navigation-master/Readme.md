# Waypoint Navigation for Earth Rover Robot
This package implements a *waypoint navigation controller* for the [earth rover](https://earthrover.cc/) robot. Using this package you can ask the robot to navigate multiple waypoints defined in _cartesian_ or _geographical_ coordinate system.

_earth_rover_navigation_ uses the [teb_local_planner](http://wiki.ros.org/teb_local_planner) and [move_base](http://wiki.ros.org/move_base) packages to control the motion of the robot.   
## Installation
Clone the repository:

	$ git clone https://github.com/earthrover/earth_rover_navigation.git

 Compile it:

 	$ roscd && cd .. && catkin_make

## Dependencies
To succesfully commpile this package you need:
1. [TooN libs](https://www.edwardrosten.com/cvd/toon.html): TooN is a C++ numerics library which is designed to operate efficiently on large numbers of small matrices, and provides easy access to a number of algorithms including matrix decompositions and optimizations. To get TooN, download the last TooN lib version and follow the installation instruction on [Documentation](https://www.edwardrosten.com/cvd/toon/html-user/index.html) page.
2. [Robohelper](https://github.com/jocacace/robohelper): Robotic Helper contains some functions to useful to work with robotic programs.
3. [teb_local_planner](http://wiki.ros.org/teb_local_planner)
4. [move_base](http://wiki.ros.org/move_base)

## Use this package

### Input
You can add new waypoints to navigate using the following ROS services:
- /earth_rover_navigation/add_cartesian_waypoint: to add waypoints specified in cartesian space
- /earth_rover_navigation/add_geo_waypoint: to add waypoints specified in geographical space

After added a waypoint, you could ask to:

- start the navigation: /earth_rover_navigation/start_navigation
- pause the navigation: /earth_rover_navigation/pause_navigation
- cancel the navigation: /earth_rover_navigation/cancel_navigation

To start the navigation, you should provide the following information:
- GPS data
- Imu data

### Parameters
Set in the launch file the following parameters:
- gps_topic: the name of the topic to read the GPS data
- imu_topic: the name of the topic to read the IMU data


### Services
- /earth_rover_navigation/add_geo_waypoint:

        [earth_rover_navigation/geo_wp_srv]:
        		sensor_msgs/NavSatFix[] gps_wp
				geometry_msgs/Quaternion[] orientation
				---
				std_msgs/Bool success


 - /earth_rover_navigation/add_cartesian_waypoint:

 		[earth_rover_navigation/cartesian_wp_srv]:
        		sensor_msgs/NavSatFix[] gps_wp
				geometry_msgs/Quaternion[] orientation
				---
				std_msgs/Bool success

 - start the navigation: /earth_rover_navigation/start_navigation

         [earth_rover_navigation/start_navigation]
                std_msgs/Empty data
                ---
                std_msgs/Bool success
- pause the navigation: /earth_rover_navigation/pause_navigation

		[earth_rover_navigation/pause_navigation]
                std_msgs/Empty data
                ---
                std_msgs/Bool success


- cancel the navigation: /earth_rover_navigation/cancel_navigation

       	[earth_rover_navigation/cancel_navigation]
                std_msgs/Empty data
                ---
                std_msgs/Bool success


## Client
To test the navigation you might use the client provided with the package. With this program, you could specify the waypoint list using text files:
1. Place the file in the ~/earth_rover_navigation/src/client/ directory
2. Specify the file name as parameter in the ~/earth_rover_navigation/launch/client.launch file:

	- csv_file: es. cartesian_wp.txt
	-
3. Run the client:

	    	$ roslaunch earth_rover_navigation client.launch
4. Request to start the navigation:

			$ rosservice call /earth_rover_navigation/start_navigation "data: {}"
