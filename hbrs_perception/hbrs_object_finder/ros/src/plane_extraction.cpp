/*
 *  CHorizontalSurfaceExtraction.cpp
 *
 *  Created on: 09.12.2010
 *      Author: Christian Mueller
 */
#include "plane_extraction.h"
#include <Eigen/StdVector>

#define THRESHOLD_JOIN_PLANES_HEIGHT 0.01f
//Difference of plane heights
//0.01 worked fine!!
//in m
#define THRESHOLD_JOIN_PLANES_DISTANCE 0.1f
//0.15f
//0.1 worked fine
//This is the distance between two planes with similiar height(see makro above). if less then join
//Please consider that objects might have the same hight and have a plane on top -> they will join, all objects within will be cut horizontally!


//with the combination  Min_PLANE_CLUSTER_SIZE=30 and MIN_PLANAR_AREA_SIZE=0.01f fast and acceptable but
//small corners of planes not joinable with main plane (too points)
//with com 5/0.001f works but slower!
#define Min_PLANE_CLUSTER_SIZE 5
//30 worked fine

#define MIN_PLANAR_AREA_SIZE 0.02f
//0.001
//0.01f worked fine
//0.02f worked fine

//Above planar surface
#define MAX_ROI_HEIGHT 5

#define DO_MULTI_PLANE false

#define NORMAL_THRESHOLD 0.5
//lab 0.9
//biba 0.5

CPlaneExtraction::CPlaneExtraction() {
	srand(time(NULL));
}

pcl::PointCloud<pcl::PointXYZRGBNormal> CPlaneExtraction::extractHorizontalSurface(
		pcl::PointCloud<pcl::PointXYZRGBNormal> &point_cloud, bool surface) {
	//ROS_DEBUG("before in %d", (int)point_cloud.points.size ());

	Eigen::Vector3f v = Eigen::Vector3f(0, 0, 1.0); //x

	pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_Inliers, cloud_projected,
			total_point_cloud;
	pcl::PointIndices inliers;
	pcl::ModelCoefficients coefficients;
	pcl::SACSegmentation<pcl::PointXYZRGBNormal> seg;

	total_point_cloud = point_cloud;

	seg.setOptimizeCoefficients(true);
	//seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
	seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setAxis(v);
	seg.setDistanceThreshold(0.2); //0.1  0.01
	seg.setProbability(0.99);
	seg.setMaxIterations(10000);

	seg.setInputCloud(
			boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal> >(
					point_cloud));
	seg.segment(inliers, coefficients);

	if (inliers.indices.size() == 0) {
		ROS_ERROR ("[extractHorizontalSurface] Could not estimate a planar model for the given dataset.");
		return cloud_projected;
	}

	pcl::ProjectInliers<pcl::PointXYZRGBNormal> proj;
	//proj.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
	proj.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	proj.setInputCloud(point_cloud.makeShared());
	proj.setModelCoefficients(
			boost::make_shared<pcl::ModelCoefficients>(coefficients));
	proj.filter(cloud_projected);

	//ROS_DEBUG("after in %d", (int)point_cloud.points.size ());

	return cloud_projected;
}

pcl::PointCloud<pcl::PointXYZRGB> CPlaneExtraction::extractHorizontalSurfaceFromNormals(
		pcl::PointCloud<pcl::PointXYZRGB> &point_cloud, bool surface) {

	Eigen::Vector3f axis = Eigen::Vector3f(1.0, 0, 0); //x

	//ROS_DEBUG("before in %d", (int)point_cloud.points.size ());
	pcl::PointCloud<pcl::PointXYZRGB> cloud_projected;
	pcl::PointIndices inliers;
	pcl::ModelCoefficients coefficients;
	pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
	pcl::PointCloud<pcl::Normal> cloud_normals;
	pcl::PointCloud<pcl::PointNormal> cloud_pointNormals;

	cloud_normals = this->toolBox.estimatingNormals(point_cloud, 10);
	//cloud_pointNormals = this->toolBox.movingLeastSquares(point_cloud,0.005f);

	seg.setOptimizeCoefficients(true);
	// seg.setModelType (pcl::SACMODEL_NORMAL_PLANE + pcl::SACMODEL_PARALLEL_PLANE);
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	//seg.setAxis(axis);
	seg.setNormalDistanceWeight(0.1); //0.1
	seg.setMaxIterations(10000); //10000
	seg.setDistanceThreshold(0.1); //0.1 //must be low to get a really restricted horizontal plane
	//seg.setProbability(0.99);

	seg.setInputCloud(
			boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(
					point_cloud));
	seg.setInputNormals(
			boost::make_shared<pcl::PointCloud<pcl::Normal> >(cloud_normals));

	seg.segment(inliers, coefficients);

	if (inliers.indices.size() == 0) {
		ROS_ERROR("[extractHorizontalSurfaceFromNormals] Could not estimate a planar model for the given dataset.");
		return cloud_projected;
	}

	pcl::ProjectInliers<pcl::PointXYZRGB> proj;
	proj.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	proj.setInputCloud(
			boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(
					point_cloud));
	proj.setModelCoefficients(
			boost::make_shared<pcl::ModelCoefficients>(coefficients));
	proj.filter(cloud_projected);

	//ROS_DEBUG("after in %d", (int)cloud_projected.points.size ());
	//pcl::copyPointCloud(point_cloud,inliers,cloud_projected);
	//point_cloud = cloud_projected;

	return cloud_projected;
}

//axis = 0 = x
//axis = 1 = y
//axis = 2 = z
std::vector<StructPlanarSurface*> CPlaneExtraction::extractMultiplePlanes(
		pcl::PointCloud<pcl::PointXYZRGBNormal> &point_cloud_normal,
		pcl::PointCloud<pcl::PointXYZRGBNormal> &planar_point_cloud_normal,
		std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > &clustered_planes,
		int axis) {
	ROS_DEBUG("[extractMultiplePlanes] extractMultiplePlanes started ... ");
	ros::Time start, finish;
	start = ros::Time::now();
	// Extraction planar point inliers
	pcl::PointIndices planar_inliers;
	pcl::ExtractIndices<pcl::PointXYZRGBNormal> extractIndices;

	if (point_cloud_normal.points.size() == 0) {
		ROS_DEBUG("[extractMultiplePlanes] Point cloud size (point_cloud_normal) is empty");
		return std::vector<StructPlanarSurface*>();
	}

	for (unsigned int iter = 0; iter < point_cloud_normal.points.size();
			iter++) {
		//std::cout<<"Point "<<point_cloud_normal.points[iter<]
		if (point_cloud_normal.points[iter].normal[axis]
				< (NORMAL_THRESHOLD * (-1))|| point_cloud_normal.points[iter].normal[axis] > NORMAL_THRESHOLD) //0.9
				{planar_inliers.indices.push_back(iter);
	}
}

	if (planar_inliers.indices.size() == 0) {
		ROS_DEBUG("[extractMultiplePlanes] planar_inliers is empty");
		return std::vector<StructPlanarSurface*>();
	}

	extractIndices.setInputCloud(point_cloud_normal.makeShared());
	extractIndices.setIndices(
			boost::make_shared<const pcl::PointIndices>(planar_inliers));
	extractIndices.filter(planar_point_cloud_normal);

	//remove if not necessary, if you change this is will affect the minClusterSize of clustering!!!!
	toolBox.subsampling(planar_point_cloud_normal, 0.02f); //0.02 0.02f worked fined //0.01
	//------------------------------------------------------
	// Clustering planar point inliers
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBNormal> euclideanClusterExtractor;
	std::vector<pcl::PointIndices> clusteredObjectIndices;
	euclideanClusterExtractor.setInputCloud(
			planar_point_cloud_normal.makeShared());
	euclideanClusterExtractor.setClusterTolerance(0.025); //(0.025);
	euclideanClusterExtractor.setMinClusterSize(Min_PLANE_CLUSTER_SIZE); //worked fine 30/0.02 sampling
	euclideanClusterExtractor.extract(clusteredObjectIndices);

	ROS_DEBUG("[extractMultiplePlanes] Number of clustered planes: %d",(int)clusteredObjectIndices.size());



	clustered_planes.clear();
	clustered_planes.resize((int) clusteredObjectIndices.size());

	for (unsigned int iterCluster = 0;
			iterCluster < clusteredObjectIndices.size(); iterCluster++) {
		pcl::copyPointCloud(planar_point_cloud_normal,
				clusteredObjectIndices.at(iterCluster),
				clustered_planes.at(iterCluster));
	}
	//------------------------------------------------------
	// Perform RANSAC clustered planar candidates

	for (unsigned int iterCluster = 0; iterCluster < clustered_planes.size();
			iterCluster++) {
		clustered_planes.at(iterCluster) = this->extractHorizontalSurface(
				clustered_planes.at(iterCluster), true);
		ROS_DEBUG("[extractMultiplePlanes] clustered planes %d : %d points", iterCluster, clustered_planes.at(iterCluster).size());
	}
	//-------------------------------------------------------
	//Check planar relations

	finish = ros::Time::now();
	std::vector<StructPlanarSurface*> surfacesPH = this->createPlanarHierarchy(
			clustered_planes, DO_MULTI_PLANE);
	ROS_DEBUG("[extractMultiplePlanes] Execution time = %lf", (finish.toSec() - start.toSec() ));
	return surfacesPH;

	//--------------------------------
}

bool compareHeights(StructPlanarSurface* i, StructPlanarSurface* j) {
	return (i->plane_height < j->plane_height);
}
std::vector<StructPlanarSurface*> CPlaneExtraction::createPlanarHierarchy(
		std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > &clustered_planes,
		bool doMultiplane) {
	ROS_DEBUG("createPlanarHierarch] started ... ");
	ros::Time start, finish;
	start = ros::Time::now();
	pcl::ConvexHull<pcl::PointXYZRGBNormal> convexHullExtractor;
	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > filtered_clustered_planes;

	std::vector<StructPlanarSurface*> planarSurfaces;
	bool isJoined = false; //whether the plane has been merged with an other due to similarity

	for (unsigned int iterCluster = 0; iterCluster < clustered_planes.size();
			iterCluster++) {
		isJoined = false;
		StructPlanarSurface* planarSurface = new StructPlanarSurface;
		planarSurface->id = iterCluster;
		planarSurface->pointCloud = clustered_planes.at(iterCluster);

		convexHullExtractor.setInputCloud(
				boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal> >(
						planarSurface->pointCloud));
		convexHullExtractor.reconstruct(planarSurface->convexHull);
		//reconstruct does not fill width and height of the pointcloud

		ROS_DEBUG("[createPlanarHierarchy] Surface %d: ConvexHull Size: %d", planarSurface->id, planarSurface->convexHull.points.size());
		planarSurface->area = toolBox.areaConvexHull2d(
				planarSurface->convexHull);
		if (planarSurface->area < MIN_PLANAR_AREA_SIZE) //minimum 0.01f area size
		{
			ROS_DEBUG("[createPlanarHierarchy] Surface %d skipped -> area=%f < %f (points %d)", iterCluster, planarSurface->area, MIN_PLANAR_AREA_SIZE, planarSurface->pointCloud.points.size());
			continue;
		}
		planarSurface->centroid = toolBox.centroidHull2d(
				planarSurface->convexHull, planarSurface->area);

		planarSurface->plane_height = toolBox.avgValuePointCloud3d(planarSurface->convexHull, 2);
		//planarSurface->plane_height = toolBox.maxValuePointCloud3d(planarSurface->convexHull, 2);

		planarSurface->ROI_height = MAX_ROI_HEIGHT; //TODO 5m? it must be set by some max height!

		planarSurface->limited_ROI_height = false;

		planarSurface->tree = boost::make_shared<
				pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> >();
		planarSurface->tree->setInputCloud(
				boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal> >(
						planarSurface->pointCloud));

		//!!check whether to join surface since they are similar (object can split surfaces due to their shadow)
		//ToDo Currently all planes on the same height are joined!!!!!
		for (unsigned int iterCheckJoin = 0;
				iterCheckJoin < planarSurfaces.size(); iterCheckJoin++) {
			if ((fabs(
					planarSurface->plane_height
							- planarSurfaces[iterCheckJoin]->plane_height)
					< THRESHOLD_JOIN_PLANES_HEIGHT)
					&& (toolBox.distanceBetweenPlane2d(*planarSurface,
							*planarSurfaces[iterCheckJoin],
							THRESHOLD_JOIN_PLANES_DISTANCE))) {

				ROS_DEBUG(
						"[createPlanarHierarchy] Joining Candidate found (%f)!", fabs(planarSurface->plane_height - planarSurfaces[iterCheckJoin]->plane_height));
				planarSurfaces[iterCheckJoin]->pointCloud +=
						planarSurface->pointCloud;
				convexHullExtractor.setInputCloud(
						boost::make_shared<
								pcl::PointCloud<pcl::PointXYZRGBNormal> >(
								planarSurfaces[iterCheckJoin]->pointCloud));
				convexHullExtractor.reconstruct(
						planarSurfaces[iterCheckJoin]->convexHull);
				//reconstruct does not fill width and height of the pointcloud

				//
				planarSurfaces[iterCheckJoin]->area = toolBox.areaConvexHull2d(
						planarSurfaces[iterCheckJoin]->convexHull);

				planarSurfaces[iterCheckJoin]->centroid =
						toolBox.centroidHull2d(
								planarSurfaces[iterCheckJoin]->convexHull,
								planarSurface->area);
				planarSurfaces[iterCheckJoin]->plane_height =
						toolBox.avgValuePointCloud3d(
								planarSurfaces[iterCheckJoin]->convexHull, 2);
				planarSurfaces[iterCheckJoin]->ROI_height = MAX_ROI_HEIGHT; //TODO 5m? it must be set by some max height!

				planarSurfaces[iterCheckJoin]->limited_ROI_height = false;

				planarSurfaces[iterCheckJoin]->tree = boost::make_shared<
						pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> >();
				planarSurfaces[iterCheckJoin]->tree->setInputCloud(
						boost::make_shared<
								pcl::PointCloud<pcl::PointXYZRGBNormal> >(
								planarSurfaces[iterCheckJoin]->pointCloud));

				isJoined = true;
				break;
			}
		}
		//----------------------
		if (!isJoined) {
			//Do filtering Joining, minArea, minDiameter...
			//join planar surface if they are on
			/*for(unsigned int iterPlanarSurfaces = 0; iterPlanarSurfaces < planarSurfaces.size(); iterPlanarSurfaces++ )
			 {
			 if( fabs(planarSurfaces.at(iterPlanarSurfaces).height - planarSurface.height) <1.0)
			 {
			 // merge
			 }

			 }*/
			//!!!!!!!!!!!ERROR REMOVE THIS!!!!
			//!!!planarSurface.convexHull.points.push_back(planarSurface.centroid);
			planarSurfaces.push_back(planarSurface);
		}
		//filtered_clustered_planes.push_back(planarSurfaces.back().convexHull);
		//clustered_planes.at(iterCluster) = planarSurfaces.back().convexHull;
		/////////////////////////////////////////
	}
	//!!!clustered_planes = filtered_clustered_planes; // this is the input parameter which is replaced by the filtered planes

	//sort plane by their heights
	//planes are sorted in descending order
	sort(planarSurfaces.begin(), planarSurfaces.end(), compareHeights);

	//if no multi plane delete all others
	if (!doMultiplane && planarSurfaces.size() > 1) {

		std::vector<StructPlanarSurface*> newPlanarSurfaces;

		//Take the last element since it is the element with the lowest height.
		//newPlanarSurfaces.push_back(planarSurfaces[planarSurfaces.size() - 1]);
		newPlanarSurfaces.push_back(planarSurfaces[0]);

		//delete all other elements
		//for (unsigned int iter = 0; iter < planarSurfaces.size() - 1; iter++) {
		for (unsigned int iter = 1; iter < planarSurfaces.size(); iter++) {
			delete planarSurfaces[iter];
			planarSurfaces[iter] = NULL;
		}
		planarSurfaces.clear();

		planarSurfaces = newPlanarSurfaces;
		newPlanarSurfaces.clear();
	}

	unsigned int numberplanarSurfaces = planarSurfaces.size();
	for (unsigned int iter = 0; iter < numberplanarSurfaces; iter++) {
		ROS_DEBUG("[createPlanarHierarchy] %d. planarSurfaces height = %f, area = %f", planarSurfaces.at(iter)->id, planarSurfaces.at(iter)->plane_height, planarSurfaces.at(iter)->area);
	}

	//check if planar surfaces are overlapping
	//since they are sorted by height we start with the surface which is lowerst
	//then we check the next surfaces in the vector which are acendingly sorted by their height.
	//the checking is done by checking whether the centroid of a upper surface is within the convex hull of the lower one
	bool overlap = false;
	for (unsigned int iterLowerPlanes = 0;
			iterLowerPlanes < numberplanarSurfaces; iterLowerPlanes++) {
		for (unsigned int iterUpperPlanes = iterLowerPlanes + 1;
				iterUpperPlanes < numberplanarSurfaces; iterUpperPlanes++) {

			//if(toolBox.pointInsideConvexHull2d(planarSurfaces[iterLowerPlanes].convexHull, planarSurfaces[iterUpperPlanes].centroid))
			//if(toolBox.overlapConvexHull2d(planarSurfaces[iterLowerPlanes],planarSurfaces[iterUpperPlanes]))
			if (toolBox.overlapConvexHull2d2(*planarSurfaces[iterLowerPlanes],
					*planarSurfaces[iterUpperPlanes])) {
				overlap = false;
				//check whether upper surfaces are overlapping; remember surfaces are sorted by size already, so if there is an overlap dont add current one
				//since current one is higher than the one added in upperPlanarSurface
				if (planarSurfaces[iterLowerPlanes]->upperPlanarSurfaces.size()
						> 0) {
					//  start = ros::Time::now();
					for (unsigned int iterOverlap = 0;
							iterOverlap
									< planarSurfaces[iterLowerPlanes]->upperPlanarSurfaces.size();
							iterOverlap++) {
						if (toolBox.overlapConvexHull2d2(
								*planarSurfaces[iterLowerPlanes]->upperPlanarSurfaces[iterOverlap],
								*planarSurfaces[iterUpperPlanes])) {
							ROS_DEBUG("[createPlanarHierarchy] %d. planarSurfaces overlap %d(%d) lower plane", planarSurfaces[iterUpperPlanes]->id, planarSurfaces[iterLowerPlanes]->upperPlanarSurfaces[iterOverlap]->id, planarSurfaces[iterLowerPlanes]->id);
							overlap = true;
							break;
						}
						// old test
						/*   pcl::PointCloud<pcl::PointXYZRGBNormal> planeConvexCentroid;

						 planeConvexCentroid = planarSurfaces[iterUpperPlanes].convexHull;
						 planeConvexCentroid.points.push_back( planarSurfaces[iterUpperPlanes].centroid);
						 if(toolBox.pointInsideConvexHull2d(planarSurfaces[iterLowerPlanes].upperPlanarSurfaces[iterOverlap].convexHull, planeConvexCentroid))
						 {
						 ROS_DEBUG("%d. planarSurfaces overlap %d(%d) lower plane", planarSurfaces[iterUpperPlanes].id,planarSurfaces[iterLowerPlanes].upperPlanarSurfaces[iterOverlap].id,planarSurfaces[iterLowerPlanes].id);
						 overlap=true;
						 break;
						 }
						 else{
						 planeConvexCentroid = planarSurfaces[iterLowerPlanes].upperPlanarSurfaces[iterOverlap].convexHull;
						 planeConvexCentroid.points.push_back(planarSurfaces[iterLowerPlanes].upperPlanarSurfaces[iterOverlap].centroid);
						 if(toolBox.pointInsideConvexHull2d(planarSurfaces[iterUpperPlanes].convexHull,planeConvexCentroid))
						 {
						 ROS_DEBUG("%d. planarSurfaces overlap %d(%d) lower plane", planarSurfaces[iterUpperPlanes].id,planarSurfaces[iterLowerPlanes].upperPlanarSurfaces[iterOverlap].id,planarSurfaces[iterLowerPlanes].id);
						 overlap=true;
						 break;
						 }
						 }*/
					}
					//	  finish = ros::Time::now();
					//  ROS_WARN("It took %lf", (finish.toSec() - start.toSec() ));
				}
				if (!overlap) { //Add upper to lower plane!
					planarSurfaces[iterLowerPlanes]->upperPlanarSurfaces.push_back(
							planarSurfaces[iterUpperPlanes]);
					ROS_DEBUG("[createPlanarHierarchy] %d. planarSurfaces is inside in %d lower plane --> %d plane added", planarSurfaces[iterUpperPlanes]->id, planarSurfaces[iterLowerPlanes]->id, planarSurfaces[iterUpperPlanes]->id);
				}
				//check if the ROI height of the lowerPlans needs to be limited
				/* unsigned int numberLowerPlanarSurfacePoints = planarSurfaces[iterLowerPlanes].pointCloud.points.size();
				 for(unsigned int iterLowerPlanePoints=0 ; iterLowerPlanePoints <numberLowerPlanarSurfacePoints*0.1f; iterLowerPlanePoints++)
				 {
				 pcl::PointXYZRGBNormal lowerPlanePoint;
				 lowerPlanePoint = planarSurfaces[iterLowerPlanes].pointCloud.points[rand()%numberLowerPlanarSurfacePoints];
				 }*/

			}
		}
	}

	finish = ros::Time::now();
	ROS_DEBUG("[createPlanarHierarchy] Execution time = %lf", (finish.toSec() - start.toSec() ));
	return planarSurfaces;
}
