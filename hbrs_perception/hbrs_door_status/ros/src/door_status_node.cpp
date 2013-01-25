/*
 * door_status_node.cpp
 *
 *  Created on: Jan 26, 2011
 *      Author: Frederik Hegger
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <hbrs_srvs/ReturnBool.h>

std::string strNodeName = "hbrs_door_status";
sensor_msgs::LaserScanConstPtr g_pLaserScanFront;
bool bIsDoorOpen = false;
double dOpeningAngle = 10.0;
double dDistanceThreshold = 1.0;


bool isDoorOpen(hbrs_srvs::ReturnBool::Request &req, hbrs_srvs::ReturnBool::Response &res)
{
	double dAngle = 0.0;
	double dSummedDistance = 0.0;
	unsigned int dCountInAngleRange = 0;

	if(g_pLaserScanFront == NULL)
		return false;

	//sum up distance in the desired opening angle
	dAngle = g_pLaserScanFront->angle_min;
	for(unsigned int i=0; i < g_pLaserScanFront->ranges.size(); ++i, dAngle += g_pLaserScanFront->angle_increment)
	{
		//check angle against desired range and only take those distance measures
		if((dAngle >= -dOpeningAngle) && (dAngle <= dOpeningAngle))
		{
			dSummedDistance += g_pLaserScanFront->ranges[i];
			++dCountInAngleRange;
		}
	}

	//calc mean
	dSummedDistance /= dCountInAngleRange;

	//check mean distance to decide weather door is open or not
	if(dSummedDistance > dDistanceThreshold)
	{
		ROS_INFO("[%s] door status: door is OPEN", strNodeName.c_str());
		res.value = true;
	}
	else
	{
		ROS_INFO("[%s] door status: door is CLOSED", strNodeName.c_str());
		res.value = false;
	}

	return true;
}


void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in)
{
	g_pLaserScanFront = msg_in;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, strNodeName);
    ros::NodeHandle nh;

    ros::Subscriber subFrontScan = nh.subscribe<sensor_msgs::LaserScan>("/scan_front", 1, laserScanCallback);
    ros::ServiceServer srvDoorStatus;
    srvDoorStatus = nh.advertiseService(strNodeName + "/door_status", isDoorOpen);

    // read params from parameter server file
    if (nh.getParam(strNodeName + "/AngularRange", dOpeningAngle) == false)
		ROS_WARN("[%s] Parameter AngularRange not specified in launch file, used default value: %lf degree", strNodeName.c_str(), dOpeningAngle);
    if (nh.getParam(strNodeName + "/DistanceThreshold", dDistanceThreshold) == false)
		ROS_WARN("[%s] Parameter DistanceThreshold not specified in launch file, used default value: %lf meter", strNodeName.c_str(), dDistanceThreshold);

    // divide degree by 2 (to have range -angle to angle+) and convert radians
    dOpeningAngle = ((dOpeningAngle / 2.0) / 180.0) * M_PI;

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
