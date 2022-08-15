#include "camera_detector/camera_detector.h"


int main(int argc, char **argv){

	ros::init(argc, argv, "camera_detector");
	
	CamDetector camera_detector;
	ros::spin();

	return 0;

}


