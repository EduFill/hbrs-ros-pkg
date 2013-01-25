#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl16/segmentation/extract_polygonal_prism_data.h>

#include <hbrs_srvs/AccumulateTabletopCloud.h>

#include "aliases.h"
#include "cloud_accumulation.h"
#include "helpers.hpp"

/** This node provides a service to accumulate parts of pointclouds that are
  * above some given planar polygon.
  *
  * In each pointcloud in the stream the algorithm extracts those points that
  * are above the given planar polygon and merges them into an occupancy octree.
  * The resulting cloud is then output to the user.
  *
  * Provides services:
  *   1) "accumulate_tabletop_cloud"
  *
  * Publishes:
  *   1) "accumulated_cloud"
  *      The cloud output as a response to the user of the service is also
  *      forwarded to this topic for the visualization purposes.
  *
  * Subscribes:
  *   1) "/camera/rgb/points"
  *      The subscription is activated on demand, i.e. when the service is idle
  *      the node unsubscribes to avoid bandwidth consumption.
  */
class TabletopCloudAccumulatorNode
{

public:

  TabletopCloudAccumulatorNode()
  {
    ros::NodeHandle nh;
    accumulate_service_ = nh.advertiseService("accumulate_tabletop_cloud", &TabletopCloudAccumulatorNode::accumulateCallback, this);
    accumulated_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("accumulated_cloud", 1);
    ROS_INFO("Service [accumulate_tabletop_cloud] started.");
  }

private:

  bool accumulateCallback(hbrs_srvs::AccumulateTabletopCloud::Request& request, hbrs_srvs::AccumulateTabletopCloud::Response& response)
  {
    ROS_INFO("Received [accumulate_tabletop_cloud] request.");
    updateConfiguration();
    PointCloud::Ptr polygon_cloud(new PointCloud);
    PlanarPolygon polygon;
    convertPlanarPolygon(request.polygon, polygon);
    polygon_cloud->points = polygon.getContour();
    eppd_.setInputPlanarHull(polygon_cloud);
    ca_ = CloudAccumulation::UPtr(new CloudAccumulation(octree_resolution_));
    ros::NodeHandle nh;
    ros::Subscriber subscriber = nh.subscribe("/camera/rgb/points", 1, &TabletopCloudAccumulatorNode::cloudCallback, this);

    // Wait some time while data is being accumulated.
    ros::Time start = ros::Time::now();
    while (ca_->getCloudCount() < accumulate_clouds_ && ros::Time::now() < start + ros::Duration(accumulation_timeout_) && ros::ok())
    {
      ros::spinOnce();
    }
    subscriber.shutdown();

    ROS_INFO("Accumulated %i clouds in %.2f seconds.", ca_->getCloudCount(), (ros::Time::now() - start).toSec());
    // Pack the response
    PointCloud cloud;
    cloud.header.frame_id = frame_id_;
    cloud.header.stamp = ros::Time::now();
    ca_->getAccumulatedCloud(cloud);
    pcl16::toROSMsg(cloud, response.cloud);

    // Forward to the "accumulated_cloud" topic (if there are subscribers)
    if (accumulated_cloud_publisher_.getNumSubscribers())
      accumulated_cloud_publisher_.publish(response.cloud);

    return ca_->getCloudCount() != 0;
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &ros_cloud)
  {
    PointCloud::Ptr cloud(new PointCloud);
    pcl16::fromROSMsg(*ros_cloud, *cloud);
    frame_id_ = ros_cloud->header.frame_id;

    pcl16::PointIndices::Ptr tabletop_indices(new pcl16::PointIndices);
    eppd_.setInputCloud(cloud);
    eppd_.segment(*tabletop_indices);

    if (tabletop_indices->indices.size() == 0)
    {
      ROS_WARN("There are no points above the provided polygon.");
      return;
    }

    PointCloud::Ptr tabletop_cloud(new PointCloud);
    pcl16::copyPointCloud(*cloud, *tabletop_indices, *tabletop_cloud);
    ca_->addCloud(tabletop_cloud);
  }

  void updateConfiguration()
  {
    ros::NodeHandle pn("~");

    // Extract polygonal prism settings
    double min_height, max_height;
    pn.param("min_height", min_height, 0.01);
    pn.param("max_height", max_height, 0.20);
    eppd_.setHeightLimits(min_height, max_height);

    // Other settings
    pn.param("accumulation_timeout", accumulation_timeout_, 10);
    pn.param("accumulate_clouds", accumulate_clouds_, 1);
    pn.param("octree_resolution", octree_resolution_, 0.0025);
  }

  pcl16::ExtractPolygonalPrismData<PointT> eppd_;
  CloudAccumulation::UPtr ca_;

  ros::ServiceServer accumulate_service_;
  ros::Publisher accumulated_cloud_publisher_;

  std::string frame_id_;
  int accumulation_timeout_;
  int accumulate_clouds_;
  double octree_resolution_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tabletop_cloud_accumulator");
  TabletopCloudAccumulatorNode tcan;
  ros::spin();
  return 0;
}

