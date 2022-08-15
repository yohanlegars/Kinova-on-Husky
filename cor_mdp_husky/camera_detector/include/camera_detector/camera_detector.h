#ifndef CAMERA_DETECTOR_H_
#define CAMERA_DETECTOR_H_


#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/Detection2D.h>
#include <string>
#include <std_srvs/Trigger.h>
#include <tuple>


class CamDetector
{
	private:
		std::string source_image_topic_;

		ros::NodeHandle n_;
		ros::Subscriber node_sub_;
		ros::Publisher node_pub_;
		ros::ServiceServer see_if_human_;

		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;
		image_transport::Publisher image_pub_;
		cv::HOGDescriptor hog_detector_;
		
		std::vector<cv::Rect> detection_boxes_;
		std::vector<double> detection_confidences_;

		std_msgs::Header header;
		std::tuple<std::vector<cv::Rect>, std::vector<double>, cv::Mat> GetBoxes(const sensor_msgs::ImageConstPtr& msg);
		void imageCallback(const sensor_msgs::ImageConstPtr& msg);

	public:
		CamDetector(); 		// Constructor

		bool checkHumanService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
};


#endif /* CAMERA_DETECTOR_H_ */
