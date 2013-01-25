#ifndef SAFE_CMD_VEL_H_
#define SAFE_CMD_VEL_H_


// ROS Includes
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

// RAW 
#include <hbrs_srvs/ReturnBool.h>

class SafeCmdVel
{
public:

	SafeCmdVel(ros::NodeHandle &n);
	~SafeCmdVel();

	bool is_robot_to_close_to_obstacle(hbrs_srvs::ReturnBool::Request &req, hbrs_srvs::ReturnBool::Response &res);

private:
	void baseCommandCallback(const geometry_msgs::Twist& desired_velocities);
	void laserFrontCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
	void laserRearCallback(const sensor_msgs::LaserScan::ConstPtr &scan);

	ros::NodeHandle nh_;

	ros::Subscriber sub_base_cmd_;
	ros::Subscriber sub_laser_front_;
	ros::Subscriber sub_laser_rear_;
	ros::Publisher pub_safe_base_cmd_;

	ros::ServiceServer srv_close_to_obstacle_;

	geometry_msgs::Twist base_vel_to_set_;


	bool is_robot_in_soft_padding_left_;
	bool is_robot_in_soft_padding_right_;
	bool is_robot_in_soft_padding_front_;
	bool is_robot_in_soft_padding_back_;
	bool is_robot_in_hard_padding_left_;
	bool is_robot_in_hard_padding_right_;
	bool is_robot_in_hard_padding_front_;
	bool is_robot_in_hard_padding_back_;

	double soft_padding_distance_;
	double soft_padding_xy_velocity_;
	double soft_padding_theta_velocity_;
	double hard_padding_distance_;

	double robot_footprint_width_;
	double robot_footprint_length_;

};

#endif /* SAFE_CMD_VEL_H_ */
