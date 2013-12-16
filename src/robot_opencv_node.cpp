#include <robot_brain/opencv.hpp>

int main( int argc, char** argv ){
  
	ros::init(argc, argv, "robot_opencv_node");
  
	robot_opencv Sparkys_OpenCV(CAMERA_INDEX);
	
	Sparkys_OpenCV.color_ =  false;
	Sparkys_OpenCV.face_ = false;


	while ( nh_.ok() ){
		Sparkys_OpenCV.cap.read(frame_);
		if( !frame.empty() ) Sparkys_OpenCV.detect_color();
		else {
			ROS_FATAL("opencv: FRAME FAIL " );
			Sparkys_OpenCV.sendError();	
		}
		
		if (color & !face ) Sparkys_OpenCV.detect_face();			//if we have a color and we havent previously detected a face we look for a face
		if(face) Sparkys_OpenCV.track_person();				//if we have a face we follow the color 
		else {							//if not 
			message.camera = 's';
			message.motor = 'f'; 
			message.errorOpenCV = '0';
			opencv_pub_.publish(opencv_msg_);
		}
		// ROS_INFO("Frames");
   }
  ROS_FATAL("opencv: NODE FAILURE");
  return -1;
}




