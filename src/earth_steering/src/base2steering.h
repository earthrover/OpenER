/* Translate cmd_vel message in a 4 steering control message:
		- Input: 
				Dot_X: 			linear velocity of the mobile base
				Dot_theta: 	angular velocity of the mobile base
*/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread.hpp"
#include "four_wheel_steering_msgs/FourWheelSteering.h"
#include "geometry_msgs/Twist.h"

class base2steering {
	public:
		base2steering();
		void run();
		void fws_ctrl();
		void cmd_vel_cb( geometry_msgs::Twist );

	private:

		ros::NodeHandle _nh;
		ros::Subscriber _cmd_vel_sub;
		ros::Publisher  _fws_vel;

		float _x_vel;
		float _y_vel;
		float _t_vel;
		double _angle_max;
		double _wheelbase;
		double _max_x_vel;
};
