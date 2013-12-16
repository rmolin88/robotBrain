#include <robot_brain/opencv.hpp>

int main( int argc, char** argv ){
  
	ros::init(argc, argv, "robot_opencv_node");
  
	robot_opencv Sparkys_OpenCV;
	
	while ( Sparkys_OpenCV.nh_.ok() ){
		Sparkys_OpenCV.cap.read(Sparkys_OpenCV.frame_);
		
		if( !Sparkys_OpenCV.frame_.empty() ) Sparkys_OpenCV.detect_color();
		else {
			ROS_FATAL("opencv: FRAME FAIL " );
			Sparkys_OpenCV.send_error();	
		}
		
		if (Sparkys_OpenCV.color_ & !Sparkys_OpenCV.face_ ) Sparkys_OpenCV.detect_face();			//if we have a color and we havent previously detected a face we look for a face
		
		if(Sparkys_OpenCV.face_) Sparkys_OpenCV.track_person();				//if we have a face we follow the color 
		else {							//if not 
			Sparkys_OpenCV.opencv_msg_.camera = 's';
			Sparkys_OpenCV.opencv_msg_.motor = 'f'; 
			Sparkys_OpenCV.opencv_msg_.errorOpenCV = '0';
			Sparkys_OpenCV.opencv_pub_.publish(Sparkys_OpenCV.opencv_msg_);
		}
		// ROS_INFO("Frames");
   }
  ROS_FATAL("opencv: NODE FAILURE");
  return -1;
}




