#include <ros/ros.h>
#include <opencv/cv.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>


struct RegionOfInterest {
	unsigned int x_offset;
	unsigned int y_offset;
	unsigned int width;
	unsigned int height;
};


class RoiExtraction {
	public:		// functions
		RoiExtraction(const std::string &camera_frame);
		virtual ~RoiExtraction();
		RegionOfInterest Extract(const sensor_msgs::PointCloud2 &cluster);


	private:	// functions
		std::vector<geometry_msgs::PointStamped> FindBoundingBox(const sensor_msgs::PointCloud2 &cluster);
		std::vector<cv::Point2f> ProjectPointsToCamera(const std::vector<geometry_msgs::PointStamped> &points) const;


	private:	// variables
	    ros::NodeHandle _nh;
	    ros::ServiceClient _bb_finder;
	    tf::TransformListener _tf_listener;
	    std::string _camera_frame;
};
