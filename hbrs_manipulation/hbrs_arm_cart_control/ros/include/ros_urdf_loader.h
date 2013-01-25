/*
 * ros_urdf_loader.h
 *
 *  Created on: Nov 29, 2012
 *      Author: matthias
 */

#ifndef ROSURDFLOADER_H_
#define ROSURDFLOADER_H_

#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

#include <ros/ros.h>
#include <urdf_interface/joint.h>

namespace arm_cc {

class ROS_URDF_Loader {
public:
	ROS_URDF_Loader();
	virtual ~ROS_URDF_Loader();

	bool loadModel(ros::NodeHandle& node_handle,
			std::string root_name,
			std::string tip_name,
			KDL::Chain& out_arm,
			std::vector<boost::shared_ptr<urdf::JointLimits> >& out_joint_limits);

};

} /* namespace arm_cc */
#endif /* ROSURDFLOADER_H_ */
