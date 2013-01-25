#include "object_segmentation.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "hbrs_object_finder");

	ObjectSegmentation* segmentation = new ObjectSegmentation();

	ros::spin();

	return 0;
}
