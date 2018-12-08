#include "base2steering.h"

using namespace std;

void load_param( double & p, double def, string name ) {
	ros::NodeHandle n_param("~");
	if( !n_param.getParam( name, p))
		p = def;
	cout << name << ": " << "\t" << p << endl;
}

base2steering::base2steering() {	
	_cmd_vel_sub = _nh.subscribe("/cmd_vel", 0, &base2steering::cmd_vel_cb, this);	
	load_param( _angle_max, 0.8, "delta_max");
	load_param( _max_x_vel, 1.0, "max_x_vel");
	load_param( _wheelbase, 1.0, "wheelbase");
	
	_fws_vel = _nh.advertise<four_wheel_steering_msgs::FourWheelSteering>("/four_wheel_steering_controller/cmd_four_wheel_steering", 0);
	twist_pub = _nh.advertise<geometry_msgs::Twist>("/four_wheel_steering_controller/cmd_vel", 0);

	_x_vel = _t_vel = _y_vel = _z_vel =  0.0;	

}

float domega_to_angle( float v, float domega, float wheelbase ) {
	if( v == 0 || domega == 0 ) 
		return 0.0;
	float radius = v / domega;
	return atan( wheelbase / radius );
}

void base2steering::cmd_vel_cb( geometry_msgs::Twist cmd ) {
	_x_vel = cmd.linear.x;
	_y_vel = cmd.linear.y;//x + y enables crab steering	
	_z_vel = cmd.linear.z;//rotation in place
	_t_vel = cmd.angular.z;// normal 4WS	
}

void base2steering::fws_ctrl() {	
	ros::Rate r(5);	
	four_wheel_steering_msgs::FourWheelSteering fws_ctrl_msg;
	geometry_msgs::Twist rotationz_msg	;

    fws_ctrl_msg.acceleration = 0.5;
    fws_ctrl_msg.jerk = 0.5;
    std::string steering_mode;
    std::string steering_joypad ("joypad");	

	float speed=0,angle=0;
	while(ros::ok()) {
        // this acts as a guard to block sending if there is a steering mode and it's set to "joypad"
	    if (ros::param::get("/steering_mode", steering_mode)){
	        if(steering_mode.compare(steering_joypad) == 0){
                r.sleep();
                continue;
	        }
        }
		speed=0;					
		if((fabs(_y_vel)>0)&&(fabs(_t_vel)==0)&&(_z_vel==0)){// Crab Steering			
			speed =sqrtf(_x_vel*_x_vel+_y_vel*_y_vel)*_max_x_vel*copysignf(1.0, _x_vel);//	mean val *max *sign		
			angle = atan2f(_y_vel*copysignf(1.0, _x_vel),fabs(_x_vel));			
			angle = ( fabs(angle) > _angle_max ) ? ( angle < 0 ) ? -_angle_max : _angle_max : angle;

			fws_ctrl_msg.front_steering_angle = angle;
			fws_ctrl_msg.rear_steering_angle  = angle;

		}else if((fabs(_x_vel)>0)&&(_z_vel==0)){//counter steering STD 4WS
			speed = _x_vel*_max_x_vel;		
			angle = domega_to_angle( _x_vel, _t_vel, _wheelbase );
			angle = ( fabs(angle) > _angle_max ) ? ( angle < 0 ) ? -_angle_max : _angle_max : angle;

			fws_ctrl_msg.front_steering_angle = angle;
			fws_ctrl_msg.rear_steering_angle = -angle;
		}
		else if((fabs(_z_vel)>0)&&(_x_vel==0)&&(_y_vel==0)&&(angle==0)){//counter steering STD 4WS
			rotationz_msg.linear.z = _z_vel;
			rotationz_msg.linear.x = 0;
			rotationz_msg.linear.y = 0;
			rotationz_msg.angular.z = 0;
			//////////////TWIST CMD
			twist_pub.publish(rotationz_msg);
    		//printf("ROT in place %f\n",_z_vel);
		}
		if (_z_vel==0){
			fws_ctrl_msg.speed=speed;
			
			_fws_vel.publish( fws_ctrl_msg );
		}
		r.sleep();
	}
}

void base2steering::run() {
	boost::thread fws_ctrl_t( &base2steering::fws_ctrl, this );
	ros::spin();
}

int main( int argc, char** argv ) {
	ros::init(argc, argv, "base2steering");
	base2steering bs;
	bs.run();
	return 0;
}
