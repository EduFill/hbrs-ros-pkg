cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#...: compiler options :........................................................

#...: gnu++0x
if(CMAKE_COMPILER_IS_GNUCXX)
    execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    if(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
        add_definitions(-std=gnu++0x)
    else(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
        message(SEND_ERROR "You need GCC version 4.6 or greater to compile this package.")
    endif(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
endif(CMAKE_COMPILER_IS_GNUCXX)

#...: treat warnings as errors and disable centain warnings
add_definitions(-Wall)
add_definitions(-Werror)
add_definitions(-Wno-error=unused-variable)
add_definitions(-Wno-error=unknown-pragmas)
add_definitions(-Wno-unknown-pragmas)
add_definitions(-Wno-deprecated)
add_definitions(-Wno-deprecated-declarations)

#...: default paths  :..........................................................

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(${PROJECT_SOURCE_DIR}/ros/include/impl)

#...: eigen :...................................................................
# eigen in needed by PCL, but for some reason this dependency is not propagated
# to the packages that depend on pcl
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})

#...: vtk libraries for pcl visualization  :....................................

set(VTK_LIBS
  libvtkCommon.so
  libvtkFiltering.so
  libvtkRendering.so
  libvtkGraphics.so
)

#...: add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#...: target executables :......................................................

#...: offline_plane_extraction_test
rosbuild_add_executable(offline_plane_extraction_test
  ros/src/offline_plane_extraction_test.cpp
)
target_link_libraries(offline_plane_extraction_test
  scene_segmentation
  ${VTK_LIBS}
)
rosbuild_link_boost(offline_plane_extraction_test
  system
)

#...: online_plane_extraction_test
rosbuild_add_executable(online_plane_extraction_test
  ros/src/online_plane_extraction_test.cpp
)
target_link_libraries(online_plane_extraction_test
  scene_segmentation
)

