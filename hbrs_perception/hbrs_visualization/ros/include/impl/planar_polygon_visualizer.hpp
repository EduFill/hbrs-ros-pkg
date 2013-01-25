#ifndef PLANAR_POLYGON_VISUALIZER_HPP
#define PLANAR_POLYGON_VISUALIZER_HPP

namespace hbrs
{

namespace visualization
{

PlanarPolygonVisualizer::PlanarPolygonVisualizer(const std::string& topic_name, const std::string& frame_id, Color color, bool check_subscribers)
: frame_id_(frame_id)
, color_(color)
, check_subscribers_(check_subscribers)
{
  ros::NodeHandle nh;
  marker_publisher_ = nh.advertise<visualization_msgs::Marker>(topic_name, 1);
}

template<typename PointT>
void PlanarPolygonVisualizer::publish(const pcl16::PlanarPolygon<PointT>& polygon)
{
  if (marker_publisher_.getNumSubscribers() == 0) return;
  visualization_msgs::Marker marker;
  buildPolygonMarker<PointT>(polygon.getContour(), marker);
  marker_publisher_.publish(marker);
}

template<typename PointT>
void PlanarPolygonVisualizer::buildPolygonMarker(const typename pcl16::PointCloud<PointT>::VectorType& points, visualization_msgs::Marker& marker, int id)
{
  if (!points.size()) return;

  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.frame_id = frame_id_;
  marker.scale.x = 0.005;
  marker.scale.y = 0.005;
  marker.color.a = 1.0;
  marker.ns = "polygon";
  marker.id = id;
  marker.color = color_;

  geometry_msgs::Point first_point;
  first_point.x = points[0].x;
  first_point.y = points[0].y;
  first_point.z = points[0].z;
  marker.points.push_back(first_point);

  for (size_t i = 1; i < points.size(); i++)
  {
    const auto& point = points[i];
    geometry_msgs::Point pt;
    pt.x = point.x;
    pt.y = point.y;
    pt.z = point.z;
    marker.points.push_back(pt);
    marker.points.push_back(pt);
  }

  marker.points.push_back(first_point);
}

}

}

#endif /* PLANAR_POLYGON_VISUALIZER_HPP */

