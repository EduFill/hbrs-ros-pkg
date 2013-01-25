#include "safe_cmd_vel.h"


SafeCmdVel::SafeCmdVel(ros::NodeHandle &n) : nh_(n)
{
	// Publisher / Subscriber
	sub_base_cmd_ = nh_.subscribe("/cmd_vel_safe", 1, &SafeCmdVel::baseCommandCallback, this);
	sub_laser_front_ = nh_.subscribe("/scan_front", 1, &SafeCmdVel::laserFrontCallback, this);
	//sub_laser_rear_ = nh_.subscribe("/scan_rear", 1, &SafeCmdVel::laserRearCallback, this);

	pub_safe_base_cmd_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	// Services
	srv_close_to_obstacle_ = nh_.advertiseService("is_robot_to_close_to_obstacle", &SafeCmdVel::is_robot_to_close_to_obstacle, this);

	// Parameter
	soft_padding_distance_ = 0.05;
	soft_padding_xy_velocity_ = 0.01;
	soft_padding_theta_velocity_ = 0.05;
	hard_padding_distance_ = 0.01;

	// TODO read in from urdf or as parameter
	robot_footprint_width_ = 0.40;
	robot_footprint_length_ = 0.60;

    if (nh_.getParam("soft_padding_distance", soft_padding_distance_) == false)
		ROS_WARN("Parameter \"soft_padding_distance\" not available on parameter server, use default value: %lf ", soft_padding_distance_);
    if (nh_.getParam("soft_padding_xy_velocity", soft_padding_xy_velocity_) == false)
    		ROS_WARN("Parameter \"soft_padding_xy_velocity\" not available on parameter server, use default value: %lf ", soft_padding_xy_velocity_);
    if (nh_.getParam("soft_padding_theta_velocity", soft_padding_theta_velocity_) == false)
    		ROS_WARN("Parameter \"soft_padding_theta_velocity\" not available on parameter server, use default value: %lf ", soft_padding_theta_velocity_);
    if (nh_.getParam("hard_padding_distance", hard_padding_distance_) == false)
    		ROS_WARN("Parameter \"hard_padding_distance\" not available on parameter server, use default value: %lf ", hard_padding_distance_);


	ROS_INFO("hbrs_safe_cmd_vel successfully initialized");
}

SafeCmdVel::~SafeCmdVel()
{
	sub_base_cmd_.shutdown();
	sub_laser_front_.shutdown();
	sub_laser_rear_.shutdown();
	pub_safe_base_cmd_.shutdown();
	srv_close_to_obstacle_.shutdown();
}

bool SafeCmdVel::is_robot_to_close_to_obstacle(hbrs_srvs::ReturnBool::Request &req, hbrs_srvs::ReturnBool::Response &res)
{
	res.value = is_robot_in_hard_padding_back_ || is_robot_in_hard_padding_front_ || is_robot_in_hard_padding_left_ || is_robot_in_hard_padding_right_;

	return true;
}

void SafeCmdVel::baseCommandCallback(const geometry_msgs::Twist& desired_velocities)
{
	base_vel_to_set_ = desired_velocities;

	//std::cout << "base vel: " << base_vel_to_set_ << " hard: " << is_robot_in_hard_padding_front_ << " soft: " << is_robot_in_soft_padding_front_ << std::endl;

	if (base_vel_to_set_.linear.x > 0.0)
	{
		if(is_robot_in_hard_padding_front_)
			base_vel_to_set_.linear.x = 0.0;

		else if(is_robot_in_soft_padding_front_)
			base_vel_to_set_.linear.x = soft_padding_xy_velocity_;

	}

	pub_safe_base_cmd_.publish(base_vel_to_set_);
}

void SafeCmdVel::laserFrontCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	double angle = scan->angle_min;
	double x = 0.0, y = 0.0;

	is_robot_in_soft_padding_front_ = false;
	is_robot_in_hard_padding_front_ = false;

	for(unsigned int i = 0; i < scan->ranges.size(); i++, angle += scan->angle_increment)
	{
		// TODO: check if there is a nicer solution
		if(scan->ranges[i] <= 0.01)
			continue;

		// convert Polar to Cartesian coordinates
		x = scan->ranges[i] * cos(angle);
		y = scan->ranges[i] * sin(angle);

		//std::cout << "x: " << x << " y: " << y << " f/2: " << (robot_footprint_width_/2) << " f/2+h: " << ((robot_footprint_width_/2) + hard_padding_distance_)

		// check front area
		if((y >= -((robot_footprint_width_/2) + hard_padding_distance_)) && (y <= ((robot_footprint_width_/2) + hard_padding_distance_)))
		{
			if(x <= hard_padding_distance_)
			{
				//std::cout << "		IN HARD LIMIT" << std::endl;
				is_robot_in_hard_padding_front_ = true;
				return;
			}
		}

		else if((y >= -((robot_footprint_width_/2) + soft_padding_distance_)) && (y <= ((robot_footprint_width_/2) + soft_padding_distance_)))
		{
			//std::cout << "check soft padding in front area. hard dist: " << hard_padding_distance_ << " x: " << x << std::endl;
			if(x <= soft_padding_distance_)
			{
				is_robot_in_soft_padding_front_ = true;
			}

		}

	}

	//if(is_robot_in_soft_padding_front_)
		//std::cout << "		IN SOFT LIMIT" << std::endl;

}
