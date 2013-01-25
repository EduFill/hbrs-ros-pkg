#ifndef CLUSTERED_POINT_CLOUD_VISUALIZER_HPP
#define CLUSTERED_POINT_CLOUD_VISUALIZER_HPP

#include <sensor_msgs/PointCloud2.h>

namespace hbrs
{

namespace visualization
{

ClusteredPointCloudVisualizer::ClusteredPointCloudVisualizer(const std::string& topic_name, const std::string& frame_id, bool check_subscribers)
: frame_id_(frame_id)
, check_subscribers_(check_subscribers)
{
  ros::NodeHandle nh;
  cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 1);
  for (size_t i = 0; i < COLORS_NUM; ++i)
  {
    COLORS[i] = 1.0 * rand() / RAND_MAX;
  }
}

template<typename PointT>
void ClusteredPointCloudVisualizer::publish(const std::vector<typename pcl16::PointCloud<PointT>::Ptr>& clusters)
{
  if (cloud_publisher_.getNumSubscribers() == 0) return;
  pcl16::PointCloud<pcl16::PointXYZRGB> composite;
  size_t color = 0;
  for (const auto& cloud : clusters)
  {
    for (const auto& point : cloud->points)
    {
      pcl16::PointXYZRGB pt;
      pt.x = point.x;
      pt.y = point.y;
      pt.z = point.z;
      pt.rgb = Color(color);
      composite.points.push_back(pt);
    }
    color++;
  }
  composite.header.frame_id = frame_id_;
  composite.width = composite.points.size();
  composite.height = 1;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl16::toROSMsg(composite, cloud_msg);
  cloud_publisher_.publish(cloud_msg);
}

}

}

#endif /* CLUSTERED_POINT_CLOUD_VISUALIZER_HPP */

