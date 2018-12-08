#include "earth_rover_nav.h"
#define d2r (M_PI / 180.0)

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void load_param(string & p, string def, string name) {
	ros::NodeHandle n_param("~");
	if (!n_param.getParam(name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

void load_param(bool & p, int def, string name) {
	ros::NodeHandle n_param("~");
	if (!n_param.getParam(name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

double distanceEarth(double lat1d, double lon1d, double lat2d, double lon2d) {
	double dlong = (lon2d - lon1d) * d2r;
	double dlat = (lat2d - lat1d) * d2r;
	double a = pow(sin(dlat / 2.0), 2) + cos(lat1d*d2r) * cos(lat2d*d2r) * pow(sin(dlong / 2.0), 2);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));
	double d = 6367 * c * 1000.0;

	cout << "Starting point: " << lat1d << " " << lon1d << endl;
	cout << "Destination point: " << lat2d << " " << lon2d << endl;
	cout << "Overall distance: " << d << endl;

	return d;
} //get GPS offset


earth_rover_nav::earth_rover_nav() {
	load_param(_wp_type, "Cartesian", "wp_type");
	load_param(_gps_topic, "/earth_gps/fix", "gps_topic");
	load_param(_imu_topic, "/imu_bosch/data", "imu_topic");
	load_param(_rpose_topic, "/four_wheel_steering_controller/odom", "rpose_topic");

	_gps_sub = _nh.subscribe(_gps_topic, 0, &earth_rover_nav::gps_cb, this);
	_imu_sub = _nh.subscribe(_imu_topic, 0, &earth_rover_nav::imu_cb, this);

	_rpose_sub = _nh.subscribe(_rpose_topic, 0, &earth_rover_nav::rpose_cb, this);

	_geo_wp_service = _nh.advertiseService("/earth_rover_navigation/add_geo_waypoint", &earth_rover_nav::geo_wp_service, this);
	_cartesian_wp_service = _nh.advertiseService("/earth_rover_navigation/add_cartesian_waypoint", &earth_rover_nav::cartesian_wp_service, this);

	_start_navigation_service = _nh.advertiseService("/earth_rover_navigation/start_navigation", &earth_rover_nav::start_navigation_service, this);
	_pause_navigation_service = _nh.advertiseService("/earth_rover_navigation/pause_navigation", &earth_rover_nav::pause_navigation_service, this);
	_cancel_navigation_service = _nh.advertiseService("/earth_rover_navigation/cancel_navigation", &earth_rover_nav::cancel_navigation_service, this);

	_move_base_cancel_action = _nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 0);

	_new_wp_req = false;
	_first_gps_data = false;
	_first_imu_data = false;
	_first_localization_data = false;
	_navigation_active = false;
	_wp_reached = false;
	_wp_error = false;

	_start_navigation = false;
	_pause_navigation = false;
	_cancel_navigation = false;

	_active_wp_p = Zeros;
	_active_wp_q = Zeros;
	_navigation_wp_p = Zeros;
	_navigation_wp_q = Zeros;
}

bool earth_rover_nav::start_navigation_service(earth_rover_navigation::start_navigation::Request &req, earth_rover_navigation::start_navigation::Response &res) {
	ROS_INFO("Start navigation request");
	_start_navigation = true;

	_pause_navigation = (_pause_navigation) ? false : false;

	res.success.data = true;
	return true;
}

bool earth_rover_nav::pause_navigation_service(earth_rover_navigation::pause_navigation::Request &req, earth_rover_navigation::pause_navigation::Response &res) {
	ROS_INFO("Pause navigation request");
	_pause_navigation = true;
	res.success.data = true;
	return true;
}

bool earth_rover_nav::cancel_navigation_service(earth_rover_navigation::cancel_navigation::Request &req, earth_rover_navigation::cancel_navigation::Response &res) {
	ROS_INFO("Cancel navigation request");
	_cancel_navigation = true;
	res.success.data = true;
	return true;
}

//---Input
bool earth_rover_nav::geo_wp_service(earth_rover_navigation::geo_wp_srv::Request &req, earth_rover_navigation::geo_wp_srv::Response &res) {
	if (!_first_gps_data) {
		ROS_ERROR("Cannot add GPS waypoint... robot doesn't receive the GPS signal!");
		return false;
	}
	geometry_msgs::Pose p;
	for (int i = 0; i < req.gps_wp.size(); i++) {
		//Geo WP -> Cartesian WP in /map frame!
		cout << "Input: " << _lat << " " << _lon << " / " << req.gps_wp[i].latitude << " " << req.gps_wp[i].longitude << endl;
		p.position.x = distanceEarth(_lat, _lon, req.gps_wp[i].latitude, _lon) + _robot_c_position[0];
		cout << "X coordinate: " << p.position.x << endl;
		p.position.y = distanceEarth(_lat, _lon, _lat, req.gps_wp[i].longitude) + _robot_c_position[1];
		cout << "Y coordinate: " << p.position.y << endl;

		p.orientation = req.orientation[i];
		_wp.push_back(p);

		cout << "added new wp: " << _wp.size() << endl;
	}
	//_new_wp_req = true;

	return true;
}

bool earth_rover_nav::cartesian_wp_service(earth_rover_navigation::cartesian_wp_srv::Request &req, earth_rover_navigation::cartesian_wp_srv::Response &res) {
	for (int i = 0; i < req.cartesian_wp.size(); i++) {
		_wp.push_back(req.cartesian_wp[i]);
	}
	//_new_wp_req = true;
	return true;
}

//---

void earth_rover_nav::gps_cb(sensor_msgs::NavSatFix gps) {
	if (!_first_gps_data) {
		printf("+ --- FIRST GPS DATA --- \n");
	}

	if (_lat != gps.latitude || _lon != gps.longitude)
		printf("+ Robot GPS Position (%2.2f, %2.2f)\n", gps.latitude, gps.longitude);

	_lat = gps.latitude;
	_lon = gps.longitude;

	_first_gps_data = true;
}

void earth_rover_nav::rpose_cb(nav_msgs::Odometry pose) {
	if (!_first_localization_data) {
		printf("+ --- FIRST LOC DATA --- \n");
	}

	if (_robot_c_position[0] != pose.pose.pose.position.x || _robot_c_position[1] != pose.pose.pose.position.y)
		printf("+ Robot Pose (%2.2f, %2.2f)\n", pose.pose.pose.position.x, pose.pose.pose.position.y);

	_first_localization_data = true;

	_robot_c_position = makeVector(pose.pose.pose.position.x, pose.pose.pose.position.y);
	_robot_orientation = makeVector(pose.pose.pose.orientation.w, pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z);
}

void earth_rover_nav::imu_cb(sensor_msgs::Imu imu) {
	if (!_first_imu_data) {
		printf("+ --- FIRST IMU DATA --- \n");

		//if (_imu_or[0] != imu.orientation.w || _imu_or[1] != imu.orientation.x || _imu_or[2] != imu.orientation.y || _imu_or[3] != imu.orientation.z)
		printf("+ Robot IMU Orientation (%2.2f, %2.2f,%2.2f, %2.2f)\n", imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z);
	}

	_imu_or = makeVector( imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z);

	_first_imu_data = true;
}

void earth_rover_nav::activeCb() {
	ROS_INFO("Goal just went active");
	if (!_navigation_active)
		_navigation_active = true;
}

// Called once when the goal becomes active

void earth_rover_nav::doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResult::ConstPtr& result) {
	if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
		_wp_reached = true;
	else
		_wp_error = true;
}

void earth_rover_nav::feedbackCb(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback) {
	geometry_msgs::PoseStamped robot_pose = feedback->base_position;

	_navigation_wp_p = makeVector(robot_pose.pose.position.x, robot_pose.pose.position.y);
	_navigation_wp_q = makeVector(robot_pose.pose.orientation.w, robot_pose.pose.orientation.x, robot_pose.pose.orientation.y, robot_pose.pose.orientation.z);
}

void earth_rover_nav::wp_supervisor() {
	printf("+ Launch supervisor \n");

	//Wait necessary staff!

	while (!_first_gps_data && !_first_localization_data) {
		printf("Wait for first localization data \n");
		usleep(10 * 1e6);
	}

	MoveBaseClient ac("move_base"); //Move base action client

	//wait for the action server to come up
	while (!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	ROS_INFO("Earth-rover Navigation ready");

	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	ros::Rate r(1);
	actionlib_msgs::GoalID goalId_msg;

	while (ros::ok()) {

		if (_start_navigation) {
			printf(" Start navigation \n");

			ROS_INFO("new_wp_req!");
			_start_navigation = false;

			if (_wp.size() > 0) {
				int _wp_index = 0;
				while (_wp_index < _wp.size() && !_cancel_navigation) {

					//Publish wp and check for its status
					_wp_reached = false;

					//goal.goal_id = _wp_index;
					goal.target_pose.pose.position.x = _wp[_wp_index].position.x;
					goal.target_pose.pose.position.y = _wp[_wp_index].position.y;
					goal.target_pose.pose.orientation = _wp[_wp_index].orientation;

					ROS_INFO("Sending goal");

					ac.sendGoal(goal,
						boost::bind(&earth_rover_nav::doneCb, this, _1, _2),
						boost::bind(&earth_rover_nav::activeCb, this),
						boost::bind(&earth_rover_nav::feedbackCb, this, _1)
						);

					//waiting for planning
					while (!_navigation_active) {
						usleep(0.1*1e6); //Consider a timout here
					}
					ROS_INFO("Planning complete!");

					_active_wp_p = makeVector(_wp[_wp_index].position.x, _wp[_wp_index].position.y);
					_active_wp_q = makeVector(_wp[_wp_index].orientation.w, _wp[_wp_index].orientation.x, _wp[_wp_index].orientation.y, _wp[_wp_index].orientation.z);

					while (!_wp_reached &&  !_cancel_navigation && !_pause_navigation) {
						cout << "Position error: " << norm(_active_wp_p - _navigation_wp_p) << endl;
						cout << "Orientation error: " << fabs(robohelper::YawFromMat(robohelper::QuatToMat(_active_wp_q)) - robohelper::YawFromMat(robohelper::QuatToMat(_navigation_wp_q))) << endl;
						sleep(1);
					} //Waiting for: destination reached - Cancel navigation mission - pause navigation mission
					if (_pause_navigation) {
						ROS_WARN("Navigation in pause");
						_move_base_cancel_action.publish(goalId_msg);
						while (_pause_navigation && !_cancel_navigation) {
							sleep(1);
						} //wait to continue navigation or to cancel the mission plan
					} // pause

					if (_cancel_navigation) {
						_move_base_cancel_action.publish(goalId_msg);
						ROS_WARN("Navigation cancelled");
						_start_navigation = false;
						_cancel_navigation = false;
						_wp.clear();
					} //End! cncael navigation
					else if (_wp_reached){
						ROS_INFO("Wp [%d] reached!", _wp_index);
						_wp_index++;
					}
					else {
						ROS_INFO("Navigation continue!");
					}

					r.sleep();
				}
			}
			else {
				ROS_WARN("Impossible to start navigation. No waypoint specified!");
			}
		} //New waypoint navitation request

		r.sleep();

	} //Unlimited Navigation!
}

void earth_rover_nav::run() {
	printf("+ NAVIGATION RUN \n");
	boost::thread wp_supervisor_t(&earth_rover_nav::wp_supervisor, this);
	ros::spin();
}

int main(int argc, char** argv) {
	printf("--------------------------------------------\n");
	printf("+ LAUNCH NAVIGATION " __DATE__ " " __TIME__ "\n");

	ros::init(argc, argv, "earth_rover_navigation");

	earth_rover_nav ern;
	ern.run();

	return 0;
}
