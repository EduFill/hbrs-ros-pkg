#include <ros/ros.h>
#include "safe_cmd_vel.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hbrs_safe_cmd_vel");
	ros::NodeHandle nh("~");

	SafeCmdVel safe_cmd_vel = SafeCmdVel(nh);

	ros::Rate looprate(15);
	while (ros::ok())
	{
		ros::spinOnce();
		looprate.sleep();
	}

	return 0;
}
