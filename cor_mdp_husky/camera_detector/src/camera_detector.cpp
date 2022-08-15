#include "camera_detector/camera_detector.h"

CamDetector::CamDetector(): it_(n_)		// Constructor
{
	source_image_topic_ = "/stream_video/camera_kinova/color/image_raw";
//	source_image_topic_ = "/realsense/color/image_raw";	// use the camera mounted on the robot base for debugging if necessary

	// Define Subscriber and publisher
	image_pub_ = it_.advertise("/camera_detector/image_raw", 10);
	image_sub_ = it_.subscribe(source_image_topic_, 10, &CamDetector::imageCallback, this);

	node_pub_ = n_.advertise<vision_msgs::Detection2DArray>("/camera_detector/detections", 10);

	this->see_if_human_ = n_.advertiseService("look_for_humans", &CamDetector::checkHumanService, this);

	hog_detector_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
}

void CamDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	ROS_INFO_STREAM("Looking at an image on :" << source_image_topic_);

	detection_boxes_.clear();
	detection_confidences_.clear();

	cv_bridge::CvImagePtr cv_ptr;
	
	vision_msgs::Detection2DArray output_msg;
	vision_msgs::Detection2D output;

	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);	// creates a copy of the image
	cv::Mat img = cv_ptr -> image;

	hog_detector_.detectMultiScale(img, detection_boxes_, detection_confidences_, 0, cv::Size(8, 8));

	if(!detection_boxes_.empty())
	{
		ROS_INFO_STREAM("Found Something!");

		for(auto& detection_box : detection_boxes_)
		{
			cv::rectangle(img, detection_box.tl(), detection_box.br(), cv::Scalar(0, 255, 0), 2);		// Draw the bounding box
			output.bbox.size_x = detection_box.br().x - detection_box.tl().x;
			output.bbox.size_y = detection_box.tl().y - detection_box.br().y;
			output.bbox.center.x = detection_box.tl().x + 0.5 * (output.bbox.size_x);
			output.bbox.center.y = detection_box.br().y + 0.5 * (output.bbox.size_y);
			output.header = header;
			output_msg.detections.push_back(output);
		}

		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
		output_msg.header = header;
		image_pub_.publish(msg);
		node_pub_.publish(output_msg);
	}
}


bool CamDetector::checkHumanService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	if(detection_boxes_.empty())
	{
		res.success = false;
		res.message = "No humans found";		
	}
	else
	{
		res.success = true;
		res.message = "Humans Detected : " + std::to_string(detection_boxes_.size());
	}
	return true;
}

