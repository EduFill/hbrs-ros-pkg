#ifndef BOUNDING_BOX_VISUALIZER_H
#define BOUNDING_BOX_VISUALIZER_H

#include <string>

#include <ros/ros.h>

#include <hbrs_msgs/BoundingBox.h>
#include "color.h"

namespace hbrs
{

namespace visualization
{

class BoundingBoxVisualizer
{

public:

  BoundingBoxVisualizer(const std::string& topic_name,
                        Color color,
                        bool check_subscribers = true);

  void publish(const hbrs_msgs::BoundingBox& box);

  void publish(const std::vector<hbrs_msgs::BoundingBox>& boxes);

private:

  ros::Publisher marker_publisher_;

  const Color color_;
  bool check_subscribers_;

};

}

}

#include "impl/bounding_box_visualizer.hpp"

#endif /* BOUNDING_BOX_VISUALIZER_H */

