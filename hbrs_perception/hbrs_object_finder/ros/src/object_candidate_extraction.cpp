/*
 *  CObjectCandidateExtraction.cpp
 *
 *  Created on: 09.12.2010
 *      Author: Christian Mueller
 */

#include "object_candidate_extraction.h"
#include <omp.h>

#define OBJECT_CLUSTER_TOLERANCE 0.034
//Tolerance how close clusters can appeare before get merge

#define THRESHOLD_POINT_ABOVE_LOWER_PLANE 0.01f
//0.01 std lab
//0.025f biba

//0.005f worked fine and stable but it can no extract cell phone height like objects
// 0.003f is fine for cell phone like but confidence is lower!!!
//This is a major stability factor for the extraction
//potential objectpoints above that threshold

#define THRESHOLD_POINT_ABOVE_UPPER_PLANE 0.01f

#define IS_PLANE_OBJECT__OBJECT_HEIGHT_THRESHOLD 0.03f
//0.03 worked fine

#define IS_PLANE_OBJECT__OBJECT_PLANE_HEIGHT_DIFFERENCE 0.05f
//this prevents the effect in shelf situtations: if the upper plane is recog as objects
//so if the value (distance between plane and upper plane) higher than threshold then the upper-plane-object is an upper plane
//0.08
//worked fine 0.12f
//0.1 worked fine without object-height threshold

//#define MIN_OBJECT_POINT_SIZE 0
//per object

//#define MIN_OBJECT_CANDIDATE_POINTS 50
//30
//total point set which will be used to extract objects

CObjectCandidateExtraction::CObjectCandidateExtraction()
{
	mlsFilering = 0.01f;
}

CObjectCandidateExtraction::CObjectCandidateExtraction(ros::NodeHandle &nh, float fDistance) 
{
	horizontalSurfaceExtractor = CPlaneExtraction();
	this->fDistance = fDistance; // std max Kinect distance
	this->mlsFilering = 0.01f;
	this->nh = nh;

	this->nh.param("threshold_points_above_lower_plane", this->threshold_point_above_lower_plane, 0.02);
	ROS_INFO_STREAM("   parameter 'threshold_points_above_lower_plane': " << this->threshold_point_above_lower_plane);
	this->nh.param("min_points_per_objects", this->min_points_per_objects, 10);
	ROS_INFO_STREAM("   parameter 'min_points_per_objects': " << this->min_points_per_objects);

	/* initialize random seed: */
	srand(time(NULL));
}

//returns a single point cloud colored objects ,
void CObjectCandidateExtraction::extractObjectCandidates(pcl::PointCloud<pcl::PointXYZRGB> &point_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> &planar_point_cloud,
		std::vector<StructPlanarSurface*> &hierarchyPlanes)
{

	ROS_DEBUG( "[extractObjectCandidates] extractObjectCandidates started ...");
	ros::Time start, start2, finish, finish2, start3, finish3;
	start = ros::Time::now();
	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > clusteredObjects;
	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > clusteredPlanes;
	pcl::PointCloud<pcl::PointXYZRGBNormal> point_cloud_RGB;
	pcl::PointIndices inliers, inliersObjects;
	pcl::PointCloud<pcl::PointXYZRGBNormal> total_point_cloud, point_cloud_normal;

	ROS_DEBUG( "[extractObjectCandidates] MovingLeastSquares started ... ");

	pcl::PointCloud<pcl::PointXYZRGBNormal> mls_cloud;

	if (mlsFilering > 0.0)
	{
		mls_cloud = this->toolBox.movingLeastSquares(point_cloud, mlsFilering); //0.01 std //CURRENT 0.01 //0.02f works good //0.01 veryfast but not good
	} else
	{
		pcl::concatenateFields(point_cloud, toolBox.estimatingNormals2(point_cloud), mls_cloud);
	}

	total_point_cloud = point_cloud_normal = mls_cloud;

	ROS_DEBUG( "[extractObjectCandidates] MovingLeastSquares done ... ");

	hierarchyPlanes = horizontalSurfaceExtractor.extractMultiplePlanes(point_cloud_normal, planar_point_cloud, clusteredPlanes, 2); //2

	if (hierarchyPlanes.empty())
	{
		ROS_DEBUG( "[extractObjectCandidates] response of extractMultiplePlanes is empty!");
		return;
	}

	unsigned int indexMaxPointsClusteredPlane = 0;
	//omp_set_num_threads(16);
	//omp_set_dynamic(0);
	//	#pragma omp parallel private(indexMaxPointsClusteredPlane)
	//	{
	//#pragma omp parallel for //nowait
	for (indexMaxPointsClusteredPlane = 0; indexMaxPointsClusteredPlane < hierarchyPlanes.size(); indexMaxPointsClusteredPlane++)
	{
		//planar_point_cloud = hierarchyPlanes[indexMaxPointsClusteredPlane].pointCloud;
		//---------------------------------------------
		// get plane hull
		start2 = ros::Time::now();

		pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_hull;
		//	pcl::ConvexHull2D<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> chull;
		//	chull.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal> > (planar_point_cloud));
		//	chull.reconstruct (cloud_hull);

		cloud_hull = hierarchyPlanes[indexMaxPointsClusteredPlane]->convexHull; //pointCloud;

		//ROS_DEBUG("[extractObjectCandidates] dZmax %f dZmin %f", dZmax,dZmin );
		pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_objects;
		cloud_objects.header = total_point_cloud.header;

		bool reject = false;
		unsigned int total_point_cloud_size = total_point_cloud.points.size();

		int chunk = total_point_cloud_size / 3;

		//unsigned int j = 0;
		//#pragma omp parallel shared (total_point_cloud,chunk) private(j)
		{
			//std::cout<<"TH " << omp_get_thread_num()<<std::endl;
			//#pragma omp parallel for schedule(dynamic,chunk) //nowait
			for (unsigned int j = 0; j < total_point_cloud_size; ++j)
			{
				//std::cout<<"Thread number....." <<omp_get_thread_num()<<std::endl;
				//here check also whether point below upper plane is
				if (toolBox.pointInsideConvexHull2d(cloud_hull, total_point_cloud.points[j])
						&& total_point_cloud.points[j].z
								> (toolBox.getNearestNeighborPlane(*hierarchyPlanes[indexMaxPointsClusteredPlane], total_point_cloud.points[j]).z + THRESHOLD_POINT_ABOVE_LOWER_PLANE))
				//(dZmax		+ THRESHOLD_POINT_ABOVE_LOWER_PLANE))//(dZmax+THRESHOLD_POINT_ABOVE_LOWER_PLANE)) //dZmax //dZmax-(dZmax*0.005)) //(((dZmax+dZmin)/2)+dZmax)/2)
				{
					reject = false;
					for (unsigned int iterUpperPlanes = 0; iterUpperPlanes < hierarchyPlanes[indexMaxPointsClusteredPlane]->upperPlanarSurfaces.size(); iterUpperPlanes++)
					{
						if (toolBox.pointInsideConvexHull2d(hierarchyPlanes[indexMaxPointsClusteredPlane]->upperPlanarSurfaces[iterUpperPlanes]->convexHull,
								total_point_cloud.points[j]) && total_point_cloud.points[j].z > hierarchyPlanes[indexMaxPointsClusteredPlane]->upperPlanarSurfaces[iterUpperPlanes]->plane_height
								+ THRESHOLD_POINT_ABOVE_UPPER_PLANE) //a little bit over surface(upperplane) in order to get a cube as a cube and not without uppersurface
{						reject = true;
						break;
					}
				}

					if (!reject)
					{
						//#pragma omp critical
						cloud_objects.points.push_back(total_point_cloud.points[j]);
					}
				}
			}
		} //PRAGMA

		cloud_objects.width = cloud_objects.points.size();
		cloud_objects.height = 1;
		//---------------------------------------------
		ROS_DEBUG( "[extractObjectCandidates] Number of object candidate points: %d.", (int)cloud_objects.points.size());

		if ((unsigned int) cloud_objects.points.size() > min_points_per_objects)
		{
			//Cluster objects
			pcl::EuclideanClusterExtraction<pcl::PointXYZRGBNormal> euclideanClusterExtractor;
			std::vector<pcl::PointIndices> clusteredObjectIndices;
			euclideanClusterExtractor.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal> >(cloud_objects));
			euclideanClusterExtractor.setClusterTolerance(OBJECT_CLUSTER_TOLERANCE); //(0.025);
			euclideanClusterExtractor.setMinClusterSize(min_points_per_objects);
			euclideanClusterExtractor.extract(clusteredObjectIndices);

			ROS_DEBUG( "[extractObjectCandidates] Number of objects clustered: %d", (int)clusteredObjectIndices.size());

			///clusteredObjects.resize((int)clusteredObjectIndices.size());

			for (unsigned int iterCluster = 0; iterCluster < clusteredObjectIndices.size(); iterCluster++)
			{
				pcl::PointCloud<pcl::PointXYZRGBNormal> foundObject;
				pcl::PointCloud<pcl::PointXYZRGBNormal> foundObjectFull;
				pcl::PointCloud<pcl::PointXYZRGBNormal> foundObjectHull;
				//pcl::copyPointCloud(cloud_objects,clusteredObjectIndices.at(iterCluster),clusteredObjects.at(iterCluster) );
				pcl::copyPointCloud(cloud_objects, clusteredObjectIndices.at(iterCluster), foundObject);
				if (!toolBox.isObjectPlane(*hierarchyPlanes[indexMaxPointsClusteredPlane], foundObject, IS_PLANE_OBJECT__OBJECT_HEIGHT_THRESHOLD,
						IS_PLANE_OBJECT__OBJECT_PLANE_HEIGHT_DIFFERENCE) && foundObject.points.size() > this->min_points_per_objects)
				{
					ROS_DEBUG( "[extractObjectCandidates] Object(%d) added", iterCluster);
					/*
					 pcl::ConvexHull2D<pcl::PointXYZRGBNormal,
					 pcl::PointXYZRGBNormal> convexHullExtractor;
					 convexHullExtractor.setInputCloud(boost::make_shared<
					 pcl::PointCloud<pcl::PointXYZRGBNormal> >(
					 foundObject));
					 convexHullExtractor.reconstruct(foundObjectHull);

					 toolBox.get3DPointsWithinHull(total_point_cloud, foundObjectHull,
					 0, toolBox.maxValuePointCloud3d(foundObject, 2)-toolBox.minValuePointCloud3d(foundObject, 2),
					 foundObjectFull);

					 */
					foundObjectFull = foundObject;
					//#pragma omp critical
					hierarchyPlanes[indexMaxPointsClusteredPlane]->clusteredObjects.push_back(foundObjectFull);

					//	pcl::PointXYZ centroid = toolBox.pointCloudCentroid(foundObjectFull);
					//#pragma omp critical
					hierarchyPlanes[indexMaxPointsClusteredPlane]->clusteredObjectsCentroids.push_back(toolBox.pointCloudCentroid(foundObjectFull));

					//REMVOE THAT!!!
					//	hierarchyPlanes[indexMaxPointsClusteredPlane]->clusteredObjects.back().points.push_back(toolBox.pointCloudCentroid(foundObjectFull));
					//	hierarchyPlanes[indexMaxPointsClusteredPlane]->clusteredObjects.back().width++;
				}
			}

			//-------------------------------------------------------------------------------------------------
			//add potential related points to objects (below the object and plane), how it is only interesting if we use it for rgb classfication

			//pcl::KdTreeANN<pcl::PointXYZRGB>::Ptr objectTree = boost::make_shared<pcl::KdTreeANN<pcl::PointXYZRGB> > ();
			//objectTree->setInputCloud(total_point_cloud);
			/*
			 for(unsigned int j=0; j < clusteredObjects.size(); ++j)
			 {
			 double dXObjectmin = 90, dYObjectmin = 90, dXObjectmax = -90, dYObjectmax=-90, dZObjectmax=-90, dZObjectmin=90;
			 double dObjectHeight;

			 for(unsigned int i=0; i < clusteredObjects[j].points.size(); ++i)
			 {
			 if(clusteredObjects[j].points[i].x < dXObjectmin)
			 dXObjectmin = clusteredObjects[j].points[i].x;
			 if(clusteredObjects[j].points[i].x > dXObjectmax)
			 dXObjectmax = clusteredObjects[j].points[i].x;

			 if(clusteredObjects[j].points[i].y < dYObjectmin)
			 dYObjectmin = clusteredObjects[j].points[i].y;
			 if(clusteredObjects[j].points[i].y > dYObjectmax)
			 dYObjectmax = clusteredObjects[j].points[i].y;

			 if(clusteredObjects[j].points[i].z < dZObjectmin)
			 dZObjectmin = clusteredObjects[j].points[i].z;
			 if(clusteredObjects[j].points[i].z > dZObjectmax)
			 dZObjectmax = clusteredObjects[j].points[i].z;
			 }

			 dObjectHeight = dZObjectmax - dZObjectmin;

			 ROS_INFO ("objects no %d -> points %d", j, (int)clusteredObjects[j].points.size());




			 bool found=false;
			 for(unsigned int k=0; k < total_point_cloud.points.size(); ++k)
			 {
			 if(total_point_cloud.points[k].x >= dXObjectmin && total_point_cloud.points[k].x <= dXObjectmax &&
			 total_point_cloud.points[k].y >= dYObjectmin && total_point_cloud.points[k].y <= dYObjectmax &&
			 total_point_cloud.points[k].z >= (dZmax)) //dZmin
			 {
			 //check if point already in point cloud
			 for(unsigned int l=0;l < clusteredObjects[j].points.size(); ++l )
			 {
			 if(clusteredObjects[j].points[l].x==total_point_cloud.points[k].x &&
			 clusteredObjects[j].points[l].y==total_point_cloud.points[k].y &&
			 clusteredObjects[j].points[l].z==total_point_cloud.points[k].z)
			 {
			 found=true;
			 break;
			 }
			 }

			 if(found==false)
			 clusteredObjects[j].points.push_back(total_point_cloud.points[k]);
			 found=false;
			 }
			 }
			 ROS_INFO ("objects no %d -> points %d", j, (int)clusteredObjects[j].points.size());
			 }
			 */
			//------------------
			//TIP:pcl->delete tabletop-> get each object pcl-> do distance intensity -> perfect object training without any texture (project object on black background so we can apply 2d/3d stuff)
			//   : then take several view, do the same, do ICP/toro or merge all points to get a 3d model for viewpoint estimation e.g.
			//   : match points from query with icp to find best fit. but we need to know before hand what kind of object we found-> so apply recognition/classification beforehand
			////
		}
		finish2 = ros::Time::now();
		ROS_DEBUG("[extractObjectCandidates] %d. plane computed - (%lf)",indexMaxPointsClusteredPlane ,(finish2.toSec() - start2.toSec() ));
	}
	//	}
	//clusteredObjectsInput = clusteredObjects;
	finish = ros::Time::now();
	ROS_DEBUG( "[extractObjectCandidates] Execution time = %lf", (finish.toSec() - start.toSec() ));
}

void CObjectCandidateExtraction::setDistance(float fDistance)
{
	this->fDistance = fDistance;
}

void CObjectCandidateExtraction::saveClusteredObjects(std::string filename)
{
	if (this->clusteredObjectsRGB.points.size() > 0)
	{
		pcl::io::savePCDFileASCII(filename, this->clusteredObjectsRGB);
	} else
	{
		ROS_DEBUG( "[saveClusteredObjects] nothing to save!");
	}
}

void CObjectCandidateExtraction::releaseObjectCandidates(std::vector<StructPlanarSurface*> &hierarchyPlanes)
{
	for (unsigned int iter = 0; iter < hierarchyPlanes.size(); ++iter)
	{
		//	for(unsigned int iterupper = 0; iterupper < hierarchyPlanes->upperPlanarSurfaces[iter].size(); ++iterupper)
		if (hierarchyPlanes[iter] != NULL)
		{
			delete hierarchyPlanes[iter];
			hierarchyPlanes[iter] = NULL;
			ROS_DEBUG("Cleaned up");
		}
	}
	hierarchyPlanes.clear();
}

/*	ROS_INFO ("Convex hull has: %d data points.", (int)cloud_hull.points.size ());

 pcl::ExtractPolygonalPrismData<pcl::PointXYZ> polygonalPrismData;
 polygonalPrismData.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (total_point_cloud));
 polygonalPrismData.setInputPlanarHull( boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (cloud_hull));
 polygonalPrismData.setHeightLimits (0.05, 0.5); //0.05
 polygonalPrismData.segment(inliersObjects);

 pcl::PointCloud<pcl::PointXYZ> cloud_objects;
 pcl::ExtractIndices<pcl::PointXYZ> extract_object_indices;
 extract_object_indices.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (total_point_cloud));
 extract_object_indices.setIndices (boost::make_shared<const pcl::PointIndices> (inliersObjects));
 extract_object_indices.filter(cloud_objects);

 //   point_cloud = cloud_objects;*/

