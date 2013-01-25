/*
 *  FILE_NAME.cpp
 *
 *  Created on: CURRENT DATE
 *      Author: YOUR NAME
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include "my_functional_class.h"

bool serviceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    std::cout << "service call triggered" << std::endl;    

	return true;
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in)
{
	std::cout << "scan received" << std::endl;
}

int main(int argc, char **argv)
{
    double my_parameter = 0;
    double data = 0;
    sensor_msgs::LaserScan my_scan;
    std_srvs::Empty empty_srv;

    MyFunctionalClass* my_func_class = new MyFunctionalClass();

    /* init ROS node with a name and a node handle*/	
    ros::init(argc, argv, "hbrs_ros_node_example");
    ros::NodeHandle nh("~");

    /* create a Publisher and a Subscriber */
    ros::Publisher pub_scan = nh.advertise<sensor_msgs::LaserScan>("my_topic", 1);
    ros::Subscriber sub_scan_front = nh.subscribe<sensor_msgs::LaserScan>("/scan_front", 1, laserScanCallback);

    /* create a Service Server and a Service Client */
    ros::ServiceServer srv_server = nh.advertiseService("my_service", serviceCallback);
    ros::ServiceClient srv_client = nh.serviceClient<std_srvs::Empty>("/another_service");;

    /* read a parameter from the parameter server */
    if (nh.getParam("my_param", my_parameter) == false)
		ROS_WARN("Parameter \"my_param\" not available on parameter server, use default value: %lf ", my_parameter);

    /* create a loop rate to let your node run only with maximum frequency, here 2Hz */
	ros::Rate loop_rate(2);

	while (ros::ok())
	{
        /* process topic callbacks and service request */
		ros::spinOnce();

        /* do your main computation */
        data = my_func_class->computeMyData(data);

        /* publish an empty scan */
        pub_scan.publish(my_scan);

        /* calling a service */
        if (srv_client.call(empty_srv))
            ROS_INFO("Service call successful");
        else
            ROS_ERROR("Failed to call service");
      
        /* wait to ensure that is not running than the predefined loop rate */
		loop_rate.sleep();
	}

	return 0;
}
