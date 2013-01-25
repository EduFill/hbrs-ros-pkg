/*
 *  CHorizontalSurfaceExtraction.h
 *
 *  Created on: 09.12.2010
 *      Author: Christian Mueller
 */
#ifndef CPlaneExtraction_H
#define CPlaneExtraction_H

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include "sensor_msgs/Image.h"

//#include "image_transport/image_transport.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
//#include "/opt/ros/cturtle/stacks/vision_opencv/cv_bridge/include/cv_bridge/CvBridge.h"

#include "pcl/ModelCoefficients.h"

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

//#include "pcl/sample_consensus/method_types.h"
//#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/features/normal_3d.h"

#include "pcl/common/common_headers.h"
#include "pcl/range_image/range_image.h"

//#include "pcl/filters/statistical_outlier_removal.h"
//#include "pcl/filters/passthrough.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/surface/convex_hull.h"
#include "pcl/segmentation/extract_polygonal_prism_data.h"

#include "pcl_ros/segmentation/extract_clusters.h"

#include "struct_planar_surface.h"
#include "toolbox_ros.h"
#include <time.h>
#include <algorithm>
#include <vector>

class CPlaneExtraction {
private:
	CToolBoxROS toolBox;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	CPlaneExtraction();
	pcl::PointCloud<pcl::PointXYZRGBNormal> extractHorizontalSurface(
			pcl::PointCloud<pcl::PointXYZRGBNormal> &point_cloud, bool surface);
	pcl::PointCloud<pcl::PointXYZRGB> extractHorizontalSurfaceFromNormals(
			pcl::PointCloud<pcl::PointXYZRGB> &point_cloud, bool surface);
	std::vector<StructPlanarSurface*> extractMultiplePlanes(
			pcl::PointCloud<pcl::PointXYZRGBNormal> &point_cloud_normal,
			pcl::PointCloud<pcl::PointXYZRGBNormal> &planar_point_cloud_normal,
			std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > &clustered_planes,
			int axis);
	std::vector<StructPlanarSurface*> createPlanarHierarchy(
			std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > &clustered_planes,
			bool doMultiplane = true);
	void setDistance(float fDistance);

};

#endif

