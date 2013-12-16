#include <robot_brain/opencv.hpp>

void robot_opencv::detect_color(){
      cv::blur(frame_, frame_blur_, cv::Size(3,3));
      cv::cvtColor(frame_blur_, imgHSV_, CV_BGR2HSV);
      cv::inRange(imgHSV_, cv::Scalar(lowH,lowS,lowV), cv::Scalar(highH,highS,highV), imgHSV_);
      momentsCalc_  = cv::moments(imgHSV_);
      area_     = momentsCalc_.m00;
	
      if(area_ > areaThres)  {
		color_ = true;
		momentsX_ = momentsCalc_.m10;
		//momentsY = momentsCalc.m01;
		posX_ = momentsX_/area_;
		//posY = momentsY/area;
	}
	else {face_ = false; color_ = false;}//if we loose the color and had a face we lost the face
	//ROS_INFO("[%f %f %f] xpos area",posX, posY, area);
      
}

void robot_opencv::detect_face(){
  //Detect faces
   cv::cvtColor( frame_, frame_gray_, CV_BGR2GRAY );
   cv::equalizeHist( frame_gray_, frame_gray_ );
   face_cascade_.detectMultiScale( frame_gray_, faces_, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );
  
   if(faces_.size()){
		face_ = true; 
		//publishing camera image
		out_msg_.image    = frame_; 
		out_msg_.encoding = sensor_msgs::image_encodings::BGR8;
		msg_ = out_msg_.toImageMsg();
		pub_.publish(msg_);
		ROS_FATAL("PERSON DETECTED");
    }
}

void robot_opencv::track_person(){	
	if( (posX_ < xMinThres) || (posX_ > xMaxThres) ){//| (posY < yMinThres) | (posY > yMaxThres) ) {//try to centered on the screen
		if(posX_ < xMinThres) 		/*ROS_INFO("Moving left");*/		opencv_msg_.camera = 'p';
		else if(posX_ > xMaxThres) 	/*ROS_INFO("Moving right");*/	opencv_msg_.camera = 'm';
		
		//if( posY_ < yMinThres) 	/*ROS_INFO("Moving Forward");*/		opencv_msg_.motor = 'f';
		//else if(posY_ > yMaxThres) /*ROS_INFO("Moving backwards");*/	opencv_msg_.motor = 'r';
		
		opencv_msg_.motor = 's'; 			
		opencv_msg_.errorOpenCV = '3';
		opencv_pub_.publish(opencv_msg_);
	}
	else {
		opencv_msg_.camera = 'k'; 
		opencv_msg_.motor = 's'; 
		opencv_msg_.errorOpenCV = '3';
		opencv_pub_.publish(opencv_msg_);
	// ROS_INFO("Target Acquired");
	}
}

void robot_opencv::send_error(){
	opencv_msg_.camera = 's';
	opencv_msg_.motor = 's';
	opencv_msg_.errorOpenCV = '1';
	opencv_pub_.publish(opencv_msg_);
	
	exit(-1);
}
