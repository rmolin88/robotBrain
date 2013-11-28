#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cstdio>
#include <iostream>


void imageCallback(const sensor_msgs::ImageConstPtr& msg);

// uint8_t c = 0;
// char buffer[50];
// bool goodImWrite = false;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  cv::namedWindow( "Patrolling Android View", CV_WINDOW_AUTOSIZE );
  cv::startWindowThread();
  
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  
  
  ros::spin();
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::imshow( "Patrolling Android View", in_msg->image);
    
    //saving image to hdd
//     sprintf(buffer,"/home/ray/Pictures/image%u.jpg",c);
//     goodImWrite = imwrite(buffer,in_msg->image);
//     if(!goodImWrite) ROS_FATAL("Remote Picture not created");
//     c++;
    
}
