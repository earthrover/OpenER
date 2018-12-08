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
//messages
#include "earth_rover_navigation/cartesian_wp_srv.h"
#include "earth_rover_navigation/geo_wp_srv.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <ros/package.h>
#include <iostream>
#include <fstream>

using namespace std;


void load_param(string & p, string def, string name) {
	ros::NodeHandle n_param("~");
	if (!n_param.getParam(name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

#define DATA_FILE_NAME "geo_wp.txt"

int main(int argc, char** argv) {
	printf("------------------------\n");
	printf(" NAVIGATION CLIENT " __DATE__  " " __TIME__ "!\n");

	ros::init(argc, argv, "eart_rover_navigation_client");

	ros::NodeHandle nh;
	ros::ServiceClient geo_client = nh.serviceClient<earth_rover_navigation::geo_wp_srv>("/earth_rover_navigation/add_geo_waypoint");
	ros::ServiceClient catesian_client = nh.serviceClient<earth_rover_navigation::cartesian_wp_srv>("/earth_rover_navigation/add_cartesian_waypoint");
	std::string csv_file;

	load_param(csv_file, DATA_FILE_NAME, "csv_file");

	std::string path = ros::package::getPath("earth_rover_navigation");

	string filename = path + "/src/client/" + csv_file;
	printf("+ Loading file %s\n", filename.c_str());

	ifstream coord_file(robohelper::string2char(filename)); // declare file stream: http://www.cplusplus.com/reference/iostream/ifstream/
	string value;
	float x, y, theta;
	string type;
	string n;
	coord_file >> n >> type;
	cout << "Type: " << type << endl;

	earth_rover_navigation::cartesian_wp_srv c_wp_srv;
	earth_rover_navigation::geo_wp_srv g_wp_srv;

	geometry_msgs::Pose p;
	sensor_msgs::NavSatFix gp;
	geometry_msgs::Quaternion gq;

	if (type == "cartesian") {
		printf(" Loading cartesian!\n");
		while (coord_file >> x >> y >> theta) {
			p.position.x = x;
			p.position.y = y;

			printf("+ CART Position (%2.2f, %2.2f) \n", x,y);

			//Yaw to quat:
			Vector<4> q = robohelper::MatToQuat(robohelper::RfromYaw(theta*M_PI / 180.0));

			p.orientation.w = q[0];
			p.orientation.x = q[1];
			p.orientation.y = q[2];
			p.orientation.z = q[3];

			printf("+ CART QUAT (%2.2f, %2.2f, %2.2f, %2.2f) \n", q[0], q[1], q[2], q[3]);

			c_wp_srv.request.cartesian_wp.push_back(p);
		}
	}
	else if (type == "geo") {
		while (coord_file >> x >> y >> theta) {
			printf("+ Load coordination file \n");
			gp.latitude = x;
			gp.longitude = y;

			printf("+ GPS Position (%2.2f, %2.2f) \n", gp.latitude, gp.longitude);

			//Yaw to quat:
			Vector<4> q = robohelper::MatToQuat(robohelper::RfromYaw(theta*M_PI / 180.0));

			gq.w = q[0];
			gq.x = q[1];
			gq.y = q[2];
			gq.z = q[3];

			printf("+ GEO QUAT (%2.2f, %2.2f, %2.2f, %2.2f) \n", q[0], q[1], q[2], q[3]);

			g_wp_srv.request.gps_wp.push_back(gp);
			g_wp_srv.request.orientation.push_back(gq);
		}
	}

	if (type == "cartesian") {
		printf("+ Call cartesian client\n");
		if (catesian_client.call(c_wp_srv)) {

		}
		else {
			cout << "Error calling service" << endl;
		}
	}
	else if (type == "geo") {
		printf("+ Call geo client\n");
		if (geo_client.call(g_wp_srv)) {

		}
		else {
			cout << "Error calling service" << endl;
		}
	}

	return 0;
}
