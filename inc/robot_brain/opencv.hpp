#ifndef OPENCV_HPP
#define OPENCV_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "robot_brain/opencv.h"


#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

#define faceColorThresh 100
#define areaThres 15000
#define xMinThres 158	
#define xMaxThres 188
#define yMinThres 170
#define yMaxThres 270
#define CAMERA_INDEX 0

#define lowH 0
#define lowS 121
#define lowV 87

#define highH 10
#define highS 255
#define highV 255

class robot_opencv {
	public:
	
	//ros variables
	ros::NodeHandle nh_; 
	sensor_msgs::ImageConstPtr msg_;
	cv_bridge::CvImage out_msg_;
	image_transport::Publisher pub_;
	image_transport::ImageTransport it_;
	robot_brain::opencv opencv_msg_;
	ros::Publisher opencv_pub_; 

	
	//program variables
	std::string face_cascade_name_;
	std::string eyes_cascade_name_;
	cv::CascadeClassifier face_cascade_;
	cv::CascadeClassifier eyes_cascade_;

	cv::Mat frame_;
	cv::Mat imgHSV_;
	cv::Mat frame_gray_;
	cv::Mat frame_blur_;
	std::vector<cv::Rect> faces_;

	cv::Moments momentsCalc_;
	float momentsX_;
	float momentsY_;
	float area_;
	float posY_;
	float posX_;
	float faceX_;
	float faceY_;
	
	bool color_;
	bool face_;

	cv::VideoCapture cap;
		
	robot_opencv(): 	it_(nh_), 
						face_cascade_name_("/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml"),  
						eyes_cascade_name_( "/usr/local/share/OpenCV/haarcascades/haarcascade_eye_tree_eyeglasses.xml"),
						color_(false),
						face_ (false)

	{
		pub_ = it_.advertise("camera/image", 1);
		opencv_pub_ = nh_.advertise<robot_brain::opencv>("opencv_commands",1000);
	  
		cap.open(CAMERA_INDEX);
		
		if(!cap.isOpened()){
			ROS_FATAL("opencv:  COULD NOT OPEN CAMERA" );
			send_error();
		}
		
		if( !face_cascade_.load( face_cascade_name_ ) ){ 
			ROS_FATAL("opencv:  COULD NOT OPEN FACE CASCADE" ); 
			send_error(); 
		}
		if( !eyes_cascade_.load( eyes_cascade_name_ ) ){ 
			ROS_FATAL("opencv:  COULD NOT OPEN EYE CASCADE"); 
			send_error(); 
		}
		
		cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
	}	

	void detect_color();
	void detect_face();
	void track_person();
	void send_error();
};




#endif /*  OPENCV_HPP  */
