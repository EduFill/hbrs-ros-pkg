#ifndef CLUSTERED_POINT_CLOUD_VISUALIZER_H
#define CLUSTERED_POINT_CLOUD_VISUALIZER_H

#include <string>

#include <ros/ros.h>

#include <pcl16/point_cloud.h>

#include "color.h"

namespace hbrs
{

namespace visualization
{

class ClusteredPointCloudVisualizer
{

public:

  ClusteredPointCloudVisualizer(const std::string& topic_name,
                                const std::string& frame_id,
                                bool check_subscribers = true);

  template<typename PointT>
  void publish(const std::vector<typename pcl16::PointCloud<PointT>::Ptr>& clusters);

  /** Fill the fields of the marker object so that it visualizes the provided
    * vector of points by drawing a polyline through them. */
  //template<typename PointT>
  //void buildPolygonMarker(const typename pcl16::PointCloud<PointT>::VectorType& points,
                          //visualization_msgs::Marker& marker,
                          //int id = 1);

private:

  ros::Publisher cloud_publisher_;

  const std::string frame_id_;
  bool check_subscribers_;

  static const size_t COLORS_NUM = 32;
  float COLORS[COLORS_NUM];

};

}

}

#include "impl/clustered_point_cloud_visualizer.hpp"

#endif /* CLUSTERED_POINT_CLOUD_VISUALIZER_H */

