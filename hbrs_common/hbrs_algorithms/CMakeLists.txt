cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)


rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
	${PROJECT_SOURCE_DIR}/..
)

rosbuild_add_library(
	hbrs_segmentation 
	
	segmentation/laserscan_segmentation.cpp
	segmentation/pointcloud_segmentation.cpp
)

rosbuild_add_library(
	hbrs_geometry 
	
	geometry/conversions.cpp
	geometry/geometric_distances.cpp
	geometry/geometric_properties.cpp	
)

rosbuild_add_library(
	hbrs_wrapper 
	
	wrapper/pcl_wrapper.cpp
)

rosbuild_add_library(
	hbrs_statistics 
	
	statistics/means.cpp
	statistics/minmax.cpp
)

rosbuild_add_library(
	hbrs_projections 
	
	projections/pointcloud_projections.cpp
)

rosbuild_add_library(
	hbrs_io 
	
	io/file_access.cpp
)

rosbuild_add_library(
	hbrs_machine_learning 
	
	machine_learning/random_trees.cpp
	machine_learning/adaboost.cpp
	machine_learning/svm.cpp
)

rosbuild_add_library(
	hbrs_data_fitting 
	
	data_fitting/circle_2d.cpp
)

rosbuild_add_library(
    hbrs_neural_networks
	
	neural_networks/self_organizing_map.cpp
)

rosbuild_add_library(
    hbrs_controller

	controller/pd_controller.cpp
	controller/pi_controller.cpp
)

