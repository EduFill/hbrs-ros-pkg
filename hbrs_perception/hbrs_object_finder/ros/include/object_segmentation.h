#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/CvBridge.h>
#include <std_srvs/Empty.h>
#include <hbrs_srvs/GetObjects.h>
#include <ros/topic.h>

#include "roi_extraction.h"
#include "toolbox_ros.h"
#include "object_candidate_extraction.h"
#include "struct_planar_surface.h"


class ObjectSegmentation {
	public:		// functions
		ObjectSegmentation();
		virtual ~ObjectSegmentation();
		void PointCloudCallback(const sensor_msgs::PointCloud2 &msg);
		bool GetObjects(hbrs_srvs::GetObjects::Request &req,
				hbrs_srvs::GetObjects::Response &res);
		bool Start(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool Stop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);


	private:	// functions
		bool PreparePointCloud(sensor_msgs::PointCloud2::ConstPtr &input, pcl::PointCloud<pcl::PointXYZRGB> &output);
		geometry_msgs::PoseStamped ExtractCentroid(const pcl::PointCloud<pcl::PointXYZRGBNormal> &object);
		IplImage *ClusterToImage(sensor_msgs::PointCloud2::ConstPtr &cluster);
		sensor_msgs::ImagePtr ExtractRegionOfInterest(const sensor_msgs::PointCloud2 &cluster, IplImage *image);
		void PublishPointClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > &cloud, ros::Publisher &publisher);



	private:	// variables
		ros::NodeHandle _nh;
		ros::Publisher _objects_points_pub;
		ros::Publisher _surfaces_points_pub;
		ros::Publisher _objects_pub;
		ros::Subscriber _cluster_sub;
		ros::ServiceServer _get_segmented_objects_srv;
		CObjectCandidateExtraction* _object_candidate_extractor;
		CToolBoxROS _tool_box;
		tf::TransformListener _tf_listener;
		std::string _node_name;
		std::string _camera_frame;
		std::string _camera_topic;
		double _downsampling_distance;

        bool _do_moving_least_square_filtering;
        double _moving_least_square_filtering_value;

		RoiExtraction* _roi_extractor;
		sensor_msgs::CvBridge _bridge;
		hbrs_srvs::GetObjects::Response _last_segmented_objects;

        bool _extract_obj_in_rgb_img;
		double _dist_min_x;
		double _dist_max_x;
		double _dist_min_y;
		double _dist_max_y;
		double _dist_min_z;
		double _dist_max_z;
};
