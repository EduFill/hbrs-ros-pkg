#include "roi_extraction.h"
#include <object_manipulation_msgs/FindClusterBoundingBox.h>
#include <sensor_msgs/point_cloud_conversion.h>


RoiExtraction::RoiExtraction(const std::string &camera_frame)
{
	_camera_frame = camera_frame;

	ROS_INFO("Waiting for 'find_cluster_bounding_box' service");
	ros::service::waitForService("find_cluster_bounding_box");
	_bb_finder = _nh.serviceClient<object_manipulation_msgs::FindClusterBoundingBox>("find_cluster_bounding_box");
	ROS_INFO("'find_cluster_bounding_box' service found");
}


RoiExtraction::~RoiExtraction()
{
}


RegionOfInterest RoiExtraction::Extract(const sensor_msgs::PointCloud2 &cluster)
{
	std::string src_frame = cluster.header.frame_id;
	std::string dst_frame = _camera_frame;
	// ros::Time time = points[i].header.stamp;
	ros::Time time = ros::Time::now();
	_tf_listener.waitForTransform(dst_frame, src_frame, time, ros::Duration(1.0));

	std::vector<geometry_msgs::PointStamped> bounding_box_3d = FindBoundingBox(cluster);
	std::vector<cv::Point2f> points = ProjectPointsToCamera(bounding_box_3d);

	unsigned int min_x = UINT_MAX;
	unsigned int max_x = 0;
	unsigned int min_y = UINT_MAX;
	unsigned int max_y = 0;
	for (unsigned int i = 0; i < points.size(); i++) {
		cv::Point2f p = points[i];

		if (p.x < min_x) min_x = p.x;
		if (p.x > max_x) max_x = p.x;
		if (p.y < min_y) min_y = p.y;
		if (p.y > max_y) max_y = p.y;
	}

	RegionOfInterest roi;
	roi.x_offset = min_x;
	roi.y_offset = min_y;
	roi.width = max_x - min_x;
	roi.height = max_y - min_y;

	return roi;
}


std::vector<geometry_msgs::PointStamped> RoiExtraction::FindBoundingBox(const sensor_msgs::PointCloud2 &cluster)
{
	// fill the request by converting PointCloud2 to PointCloud
	object_manipulation_msgs::FindClusterBoundingBox srv;
	sensor_msgs::convertPointCloud2ToPointCloud(cluster, srv.request.cluster);

	_bb_finder.call(srv);
	ROS_ASSERT(srv.response.error_code == object_manipulation_msgs::FindClusterBoundingBox::Response::SUCCESS);

	// the eight vertices that make up the box
	std::vector<geometry_msgs::PointStamped> vertices(8);

	for (unsigned int i = 0; i < vertices.size(); i++) vertices[i].header = cluster.header;

	double factor_x[] = { -0.5, 0.5 };
	double factor_y[] = { -0.5, 0.5 };
	double factor_z[] = { -0.5, 0.5 };
	int index = 0;

	// create all combinations of directions to generate the vertices
	for (unsigned int i = 0; i < 2; i++) {
		for (unsigned int j = 0; j < 2; j++) {
			for (unsigned int k = 0; k < 2; k++) {
				vertices[index].point.x = srv.response.pose.pose.position.x + (factor_x[i] * (srv.response.box_dims.x + 0.05));
				vertices[index].point.y = srv.response.pose.pose.position.y + (factor_y[j] * (srv.response.box_dims.y + 0.05));
				vertices[index].point.z = srv.response.pose.pose.position.z + (factor_z[k] * (srv.response.box_dims.z + 0.05));

				index++;
			}
		}
	}

	return vertices;
}


std::vector<cv::Point2f> RoiExtraction::ProjectPointsToCamera(const std::vector<geometry_msgs::PointStamped> &points) const
{
	double dCamera[3][3] = {
			{ 525.0,   0.0, 319.5 },
			{   0.0, 525.0, 239.5 },
			{   0.0,   0.0,   1.0 }
	};
	double dDistortion[5] = {
			0.0, 0.0, 0.0, 0.0, 0.0
	};
	double dRotationVector[3] = { 0.0, 0.0, 0.0 };
	double dTranslations[3] = { 0.0, 0.0, 0.0 };

	cv::Mat camera_matrix = cv::Mat(3, 3, CV_64F, dCamera);
	cv::Mat rotation_vector = cv::Mat(1, 3, CV_64F, dRotationVector);
	cv::Mat translation_vector = cv::Mat(1, 3, CV_64F, dTranslations);
	cv::Mat distortion = cv::Mat(1, 5, CV_64F, dDistortion);



	// ROS to OpenCV
	std::vector<cv::Point3f> model_points;		// input points from point cloud
	for (unsigned int i = 0; i < points.size(); i++) {
		geometry_msgs::PointStamped ps;

		// transform point to camera frame
		_tf_listener.transformPoint(_camera_frame, points[i], ps);

		cv::Point3f p;
		p.x = ps.point.x;
		p.y = ps.point.y;
		p.z = ps.point.z;

		model_points.push_back(p);
	}

	cv::Mat mat_model_points = cv::Mat(model_points);
	std::vector<cv::Point2f> projected_points(model_points.size());		// output 2d points


	// project points from the point cloud ""
	cv::projectPoints(mat_model_points,		// 3d points
			rotation_vector,				// rotation vector
			translation_vector,				// translation vector
			camera_matrix,					// intrinsic matrix
			distortion,						// distortion coeffs
			projected_points				// 2d projected points
	);

	return projected_points;
}
