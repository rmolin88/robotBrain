#include <ros/ros.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

void colorDetect();

cv::Mat frame;
cv::Mat imgHSV;
cv::Mat frame_blur;

//divide hue by half and give it +- 5 range


#define lowH 0
#define highH 5

#define lowS 100
#define highS 255

#define lowV 100
#define highV 255

char key;


int main( int argc, char** argv )
{

	ros::init(argc, argv, "calibrate");
	ros::NodeHandle n;

	cv::VideoCapture cap(0);
  
  if(!cap.isOpened()){
		ROS_FATAL("opencv:  COULD NOT OPEN CAMERA" );
		exit(-1);
	}
cv::namedWindow( "Patrolling Android View", CV_WINDOW_AUTOSIZE );
while ( n.ok() ){
      cap.read(frame);
      if( !frame.empty() ) colorDetect();//detectAndDisplay(); 
      else {
	ROS_FATAL("opencv: FRAME FAIL " );
	exit(-1);	
      }
	key = cv::waitKey(1);
	cv::imshow( "Patrolling Android View", imgHSV); //frame imgHSV
	if (key == 'q') exit(0);
	//do{}while(!(key=='c'));

}
}
void colorDetect(){
	cv::blur(frame, frame_blur, cv::Size(3,3));
      cv::cvtColor(frame_blur, imgHSV, CV_BGR2HSV);
      cv::inRange(imgHSV, cv::Scalar(lowH,lowS,lowV), cv::Scalar(highH,highS,highV), imgHSV);
}
