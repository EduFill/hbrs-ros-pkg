/*
 *  CObjectCandidateExtraction.h
 *
 *  Created on: 09.12.2010
 *      Author: Christian Mueller
 */

#ifndef COBJECT_CANDIDATE_EXTRACTION_H
#define COBJECT_CANDIDATE_EXTRACTION_H

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/extract_indices.h"
#include "pcl_ros/segmentation/extract_clusters.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/surface/convex_hull.h"
#include "pcl/segmentation/extract_polygonal_prism_data.h"

#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
//#include "/opt/ros/cturtle/stacks/vision_opencv/cv_bridge/include/cv_bridge/CvBridge.h"

#include "pcl/ModelCoefficients.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/features/normal_3d.h"

#include "pcl/common/common_headers.h"
#include "pcl/range_image/range_image.h"

#include "struct_planar_surface.h"
#include "toolbox_ros.h"
#include "plane_extraction.h"

#include <iostream>
#include <cstdlib>
#include <vector>

#include <stdio.h>
#include <time.h>
#include <limits>
#include <omp.h>

#include <string>

class CObjectCandidateExtraction
{

private:
	CToolBoxROS toolBox;
	CPlaneExtraction horizontalSurfaceExtractor;
    ros::NodeHandle nh;
	double threshold_point_above_lower_plane;
	int min_points_per_objects;

	float fDistance; /*max distance from camera*/
	double mlsFilering;
	float dZAxisOffSet; /*distance(height) to compensate difference between object and plane*/

	std::vector<pcl::PointCloud<pcl::PointXYZRGB> > clusteredObjects; /*last extracted object candidates*/
	pcl::PointCloud<pcl::PointXYZRGB> clusteredObjectsRGB;

public:
	CObjectCandidateExtraction();
    CObjectCandidateExtraction(ros::NodeHandle &nh, float fDistance);
	void extractObjectCandidates(pcl::PointCloud<pcl::PointXYZRGB> &point_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> &planar_point_cloud, std::vector<StructPlanarSurface*> &hierarchyPlanes);
	void releaseObjectCandidates(std::vector<StructPlanarSurface*> &hierarchyPlanes);
	void saveClusteredObjects(std::string filename);
	void setDistance(float fDistance);
	void setMlsFiltering(double setMls)
	{
		mlsFilering = setMls;
	}
};

#endif
