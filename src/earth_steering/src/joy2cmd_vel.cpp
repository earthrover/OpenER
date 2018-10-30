#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "geometry_msgs/Twist.h"

using namespace std;

float x_vel;
float t_vel;

void joy_cb( sensor_msgs::Joy joy_msg ) {
	x_vel = joy_msg.axes[1];
	t_vel = joy_msg.axes[0];
}


int main( int argc, char** argv ) {

	ros::init( argc, argv, "joyt2cmdvel");

	ros::NodeHandle nh;
	ros::Subscriber joy_sub;
	ros::Publisher cmd_vel_pub;

	joy_sub = nh.subscribe( "/joy", 0, joy_cb );
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>( "/cmd_vel", 0);

	x_vel = t_vel = 0.0;
	geometry_msgs::Twist cmd_vel;
	ros::Rate r(5);

	while( ros::ok() ) {

		cmd_vel.linear.x = x_vel;
		cmd_vel.angular.z = t_vel;	
		cmd_vel_pub.publish ( cmd_vel );
		 
		ros::spinOnce();
		r.sleep();
	}	


	return 0;
}
