#ifndef ALIASES_H_
#define ALIASES_H_

#include <vector>

#include <pcl16/point_types.h>
#include <pcl16/point_cloud.h>
#include <pcl16/geometry/planar_polygon.h>
#include <pcl16/segmentation/planar_region.h>

typedef pcl16::PointXYZRGB PointT;
typedef pcl16::Normal PointNT;
typedef pcl16::Label PointLT;

typedef pcl16::PointCloud<PointT> PointCloud;
typedef pcl16::PointCloud<PointNT> PointCloudN;
typedef pcl16::PointCloud<PointLT> PointCloudL;

typedef pcl16::PlanarPolygon<PointT> PlanarPolygon;
typedef std::vector<PlanarPolygon, Eigen::aligned_allocator<PlanarPolygon>> PlanarPolygonVector;
typedef boost::shared_ptr<PlanarPolygon> PlanarPolygonPtr;
typedef boost::shared_ptr<const PlanarPolygon> PlanarPolygonConstPtr;

typedef pcl16::PlanarRegion<PointT> PlanarRegion;
typedef std::vector<PlanarRegion, Eigen::aligned_allocator<PlanarRegion>> PlanarRegionVector;

#endif
