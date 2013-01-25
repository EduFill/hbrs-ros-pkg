#include <pcl16/octree/octree_impl.h>

#include "cloud_accumulation.h"

CloudAccumulation::CloudAccumulation(double resolution)
: resolution_(resolution)
{
  reset();
}

void CloudAccumulation::addCloud(const PointCloud::ConstPtr& cloud)
{
  octree_->setOccupiedVoxelsAtPointsFromCloud(cloud);
  cloud_count_++;
}

void CloudAccumulation::getAccumulatedCloud(PointCloud& cloud)
{
  octree_->getOccupiedVoxelCentersWithColor(cloud.points);
  cloud.width = cloud.points.size();
  cloud.height = 1;
}

void CloudAccumulation::reset()
{
  octree_ = OctreeUPtr(new Octree(resolution_));
  cloud_count_ = 0;
}

