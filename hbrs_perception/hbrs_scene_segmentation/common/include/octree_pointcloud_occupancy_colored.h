#ifndef OCTREE_POINTCLOUD_OCCUPANCY_COLORED_H
#define OCTREE_POINTCLOUD_OCCUPANCY_COLORED_H

#include <pcl16/point_types.h>
#include <pcl16/point_cloud.h>
#include <pcl16/octree/octree_pointcloud.h>
#include <pcl16/octree/octree_base.h>

using namespace pcl16::octree;

template<typename PointT = pcl16::PointXYZRGB, typename LeafContainerT = OctreeContainerDataT<uint32_t>, typename BranchContainerT = OctreeContainerEmpty<uint32_t>>
class OctreePointCloudOccupancyColored : public OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeBase<uint32_t, LeafContainerT, BranchContainerT>>
{

public:

  OctreePointCloudOccupancyColored(const double resolution)
  : OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeBase<uint32_t, LeafContainerT, BranchContainerT>>(resolution)
  {
  }

  virtual ~OctreePointCloudOccupancyColored() { };

  void setOccupiedVoxelAtPoint(const PointT& point)
  {
    OctreeKey key;
    adoptBoundingBoxToPoint(point);
    genOctreeKeyforPoint(point, key);
    this->addData(key, point.rgba);
  }

  void setOccupiedVoxelsAtPointsFromCloud(const typename pcl16::PointCloud<PointT>::ConstPtr& cloud)
  {
    for (size_t i = 0; i < cloud->points.size(); i++)
      if (isFinite(cloud->points[i]))
        this->setOccupiedVoxelAtPoint(cloud->points[i]);
  }

  uint32_t getVoxelColorAtPoint(const PointT& point) const
  {
    uint32_t color = 0;
    OctreeContainerDataT<uint32_t>* leaf = this->findLeafAtPoint(point);
    if (leaf)
      leaf->getData(color);
    return color;
  }

  void getOccupiedVoxelCentersWithColor(typename pcl16::PointCloud<PointT>::VectorType& points)
  {
    this->getOccupiedVoxelCenters(points);
    for (size_t i = 0; i < points.size(); i++)
    {
      uint32_t color = this->getVoxelColorAtPoint(points[i]);
      points[i].rgba = color;
    }
  }

};

#endif /* OCTREE_POINTCLOUD_OCCUPANCY_COLORED_H */

