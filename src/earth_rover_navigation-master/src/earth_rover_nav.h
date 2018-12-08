#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "boost/thread.hpp"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "boost/thread.hpp"
#include "geometry_msgs/Twist.h"
#include "TooN/TooN.h"
#include "robohelper/robohelper.hpp"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
//messages
#include "earth_rover_navigation/cartesian_wp_srv.h"
#include "earth_rover_navigation/geo_wp_srv.h"
#include "earth_rover_navigation/start_navigation.h"
#include "earth_rover_navigation/cancel_navigation.h"
#include "earth_rover_navigation/pause_navigation.h"

#include "move_base_msgs/MoveBaseActionFeedback.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "actionlib_msgs/GoalID.h"

using namespace std;

class earth_rover_nav {

public:
	earth_rover_nav();
	void wp_supervisor();
	void run();

	//---Raw input
	void gps_cb(sensor_msgs::NavSatFix gps);
	void imu_cb(sensor_msgs::Imu imu);
	//---

	//Odometry input (estimated pose of the robot)
	void rpose_cb(nav_msgs::Odometry pose);
	bool geo_wp_service(earth_rover_navigation::geo_wp_srv::Request &req, earth_rover_navigation::geo_wp_srv::Response &res);
	bool cartesian_wp_service(earth_rover_navigation::cartesian_wp_srv::Request &req, earth_rover_navigation::cartesian_wp_srv::Response &res);

	void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResult::ConstPtr& result);
	void feedbackCb(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback);
	void activeCb();


	bool start_navigation_service(earth_rover_navigation::start_navigation::Request &req, earth_rover_navigation::start_navigation::Response &res);
	bool pause_navigation_service(earth_rover_navigation::pause_navigation::Request &req, earth_rover_navigation::pause_navigation::Response &res);
	bool cancel_navigation_service(earth_rover_navigation::cancel_navigation::Request &req, earth_rover_navigation::cancel_navigation::Response &res);

	//ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());

private:

	ros::NodeHandle _nh;
	ros::Subscriber _imu_sub;
	ros::Subscriber _gps_sub;
	ros::Subscriber _rpose_sub;

	ros::Publisher _move_base_cancel_action;
	ros::ServiceServer _geo_wp_service;
	ros::ServiceServer _cartesian_wp_service;
	ros::ServiceServer _start_navigation_service;
	ros::ServiceServer _pause_navigation_service;
	ros::ServiceServer _cancel_navigation_service;

	bool _new_wp_req;
	volatile bool _first_gps_data;
	volatile bool _first_imu_data;
	volatile bool _first_localization_data;

	double _lat;
	double _lon;

	//Params
	string _wp_type; //Geo or Cartesian
	string _gps_topic;
	string _imu_topic;
	string _rpose_topic;

	int _wp_index;

	std::vector< geometry_msgs::Pose > _wp;

	Vector<2> _robot_c_position;
	Vector<4> _robot_orientation;
	Vector<4> _imu_or;

	//State control
	bool _navigation_active;
	bool _wp_reached;
	bool _wp_error;

	Vector<2> _active_wp_p;
	Vector<4> _active_wp_q;

	Vector<2> _navigation_wp_p;
	Vector<4> _navigation_wp_q;

	//Service data
	bool _start_navigation;
	bool _pause_navigation;
	bool _cancel_navigation;


};
