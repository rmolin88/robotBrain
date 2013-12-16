#include <robot_brain/opencv.hpp>

void detect_color(){
      cv::blur(frame_, frame_blur_, cv::Size(3,3));
      cv::cvtColor(frame_blur_, imgHSV_, CV_BGR2HSV);
      cv::inRange(imgHSV_, cv::Scalar(lowH,lowS,lowV), cv::Scalar(highH,highS,highV), imgHSV);
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

void detect_face(){
  //Detect faces
   cv::cvtColor( frame_, frame_gray_, CV_BGR2GRAY );
   cv::equalizeHist( frame_gray_, frame_gray_ );
   face_cascade.detectMultiScale( frame_gray_, faces_, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );
  
   if(faces.size()){
		face = true; 
		//publishing camera image
		out_msg_.image    = frame_; //frame
		out_msg_.encoding = sensor_msgs::image_encodings::BGR8;
		msg_ = out_msg_.toImageMsg();
		pub.publish(msg_);
		ROS_FATAL("PERSON DETECTED");
    }
}

void track_person(){	
	if( (posX < xMinThres) || (posX > xMaxThres) ){//| (posY < yMinThres) | (posY > yMaxThres) ) {//try to centered on the screen
		if(posX < xMinThres) 		/*ROS_INFO("Moving left");*/message.camera = 'p';
		else if(posX > xMaxThres) 	/*ROS_INFO("Moving right");*/message.camera = 'm';
		
		//if( posY < yMinThres) 	/*ROS_INFO("Moving Forward");*/message.motor = 'f';
		//else if(posY> yMaxThres) /*ROS_INFO("Moving backwards");*/message.motor = 'r';
		
		message.motor = 's'; 			
		message.errorOpenCV = '3';
		opencv_pub_.publish(opencv_msg_);
	}
	else {
		message.camera = 'k'; 
		message.motor = 's'; 
		message.errorOpenCV = '3';
		opencv_pub_.publish(opencv_msg_);
	// ROS_INFO("Target Acquired");
	}
}

void send_error(){
	message.camera = 's';
	message.motor = 's';
	message.errorOpenCV = '1';
	opencv_pub_.publish(opencv_msg_);
	
	exit(-1);
}
