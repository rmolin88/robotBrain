#include <ros/ros.h>
#include "robotBrain/opencvMessage.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

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

#define lowH 0
#define lowS 100
#define lowV 100

#define highH 0
#define highS 255
#define highV 255

robotBrain::opencvMessage message;
ros::Publisher opencvCommands; 
sensor_msgs::ImageConstPtr msg;
cv_bridge::CvImage out_msg;
image_transport::Publisher pub;

void detectAndDisplay();
void colorDetect();
void faceDetect();
void personTracking();
void sendError();


std::string face_cascade_name = "/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml";
std::string eyes_cascade_name = "/usr/local/share/OpenCV/haarcascades/haarcascade_eye_tree_eyeglasses.xml";
cv::CascadeClassifier face_cascade;
cv::CascadeClassifier eyes_cascade;

cv::Mat frame;
cv::Mat imgHSV;
cv::Mat frame_gray;
cv::Mat frame_blur;
std::vector<cv::Rect> faces;

cv::Moments momentsCalc;
float momentsX;
float momentsY;
float area;
float posY;
float posX;
float faceX;
float faceY;
uint8_t i =0;

uint8_t hue_low = 0;
uint8_t hue_high = 0;
uint8_t sat_low = 100; 
uint8_t sat_high = 255;
uint8_t val_low = 100;
uint8_t val_high = 255;

//uint8_t c = 0;
//char buffer[50];
//bool goodImWrite = false;
bool color =  false;
bool face = false;
//namedWindow()

int main( int argc, char** argv )
{
  ros::init(argc, argv, "face_detect");
  ros::NodeHandle n;
  
  opencvCommands = n.advertise<robotBrain::opencvMessage>("opencv_commands",1000);
  image_transport::ImageTransport it(n);
  pub = it.advertise("camera/image", 1);
  
  cv::VideoCapture cap(0);
  
  if(!cap.isOpened()){
		ROS_FATAL("opencv:  COULD NOT OPEN CAMERA" );
		sendError();
	}
  
  //Load the cascades
	
  if( !face_cascade.load( face_cascade_name ) ){ 
    ROS_FATAL("--(!)Error loading FACE CASCADE(!)--" ); 
    sendError(); 
  }
  if( !eyes_cascade.load( eyes_cascade_name ) ){ 
    ROS_FATAL("--(!)Error loading EYE CASCADE(!)--"); 
    sendError(); 
  }
  
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
  char key;
    while ( n.ok() ){
      cap.read(frame);
      if( !frame.empty() ) colorDetect();//detectAndDisplay(); 
      else {
	ROS_FATAL("opencv: FRAME FAIL " );
	sendError();	
      }
	//showing image
	cv::namedWindow( "Patrolling Android View", CV_WINDOW_AUTOSIZE );
  	cv::startWindowThread();
	cv::imshow( "Patrolling Android View", imgHSV);
      if (color & !face ) faceDetect();			//if we have a color and we havent previously detected a face we look for a face
      
      if(face) personTracking();				//if we have a face we follow the color 
      else {							//if not 
	message.camera = 's';
	message.errorOpenCV = '0';
	opencvCommands.publish( message );
      }
      key = cv::waitKey(100);
switch (key)
        {
            //hue
            case 'q':
                if (hue_low == hue_high)
                    ROS_INFO("hue_low must be less than hue_high");
                else 
                hue_low = hue_low + 1;
            break;
            case 'a':
                if (hue_low == 0) 
                    ROS_INFO("Hue is minimum");
                else
                    hue_low = hue_low - 1;
            break;
            case 'w':
                if (hue_high == 255)
                    ROS_INFO("Hue is maximum");
                else
                    hue_high = hue_high + 1;
            break;
            case 's':
                if (hue_high == hue_low)
                    ROS_INFO("hue_high must be greater than hue_low");
                else
                    hue_high = hue_high - 1;
            break;
           
            //saturation 
            case 'e':
                if (sat_low == sat_high)
                    ROS_INFO("sat_low must be less than sat_high");
                else 
                sat_low = sat_low + 1;
            break;
            case 'd':
                if (sat_low == 0) 
                    ROS_INFO("sat is minimum");
                else
                    sat_low = sat_low - 1;
            break;
            case 'r':
                if (sat_high == 255)
                    ROS_INFO("sat is maximum");
                else
                    sat_high = sat_high + 1;
            break;
            case 'f':
                if (sat_high == sat_low)
                    ROS_INFO("sat_high must be greater than sat_low");
                else
                    sat_high = sat_high - 1;
            break;
            
            //value 
            case 't':
                if (val_low == val_high)
                    ROS_INFO("val_low must be less than val_high");
                else 
                val_low = val_low + 1;
            break;
            case 'g':
                if (val_low == 0) 
                    ROS_INFO("val is minimum");
                else
                    val_low = val_low - 1;
            break;
            case 'y':
                if (val_high == 255)
                    ROS_INFO("val is maximum");
                else
                    val_high = val_high + 1;
            break;
            case 'h':
                if (val_high == val_low)
                    ROS_INFO("val_high must be greater than val_low");
                else
                    val_high = val_high - 1;
            break;
        }
      //ROS_INFO("Frames");
	ROS_INFO("Hue: %d-%d\tSat: %d-%d\tVal: %d-%d\n", hue_low, hue_high, sat_low, sat_high, val_low, val_high);
   }
  ROS_FATAL("COULD NOT CAPTURE FRAME");
  return -1;
}



void colorDetect(){
      cv::blur(frame, frame_blur, cv::Size(3,3));
      cv::cvtColor(frame_blur, imgHSV, CV_BGR2HSV);
      cv::inRange(imgHSV, cv::Scalar(hue_low,sat_low,val_low), cv::Scalar(hue_high,sat_high,val_high), imgHSV);
      momentsCalc  = cv::moments(imgHSV);
      area     = momentsCalc.m00;
	
      //
      if(area > areaThres)  {
	color = true;
	momentsX = momentsCalc.m10;
	//momentsY = momentsCalc.m01;
	posX = momentsX/area;
	//posY = momentsY/area;
	}
	else {face = false; color = false;}//if we loose the color and had a face we lost the face
	ROS_INFO("[%f %f] xpos area",posX, area);
      
}

void faceDetect(){
  //Detect faces
   cv::cvtColor( frame, frame_gray, CV_BGR2GRAY );
   cv::equalizeHist( frame_gray, frame_gray );
   face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );
  
   //for each face draws an ellipse arround and look for the red color at a distance from 
   for( i = 0; i < faces.size(); i++ )
    {
      cv::Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
      cv::ellipse( frame, center, cv::Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, cv::Scalar( 0, 255, 0 ), 2, 8, 0 );
      faceX = (float) faces[i].x;
      faceY = (float) faces[i].y;
     
      if( ((faceX + faceColorThresh) > (posX ) | (faceX - faceColorThresh) < (posX )) ) {
	face = true; 
	//publishing camera image
	out_msg.image    = frame; //frame
	out_msg.encoding = sensor_msgs::image_encodings::BGR8;
	msg = out_msg.toImageMsg();
	pub.publish(msg);
	ROS_FATAL("PERSON DETECTED");
	break;
      }
    }
}

void personTracking(){	
		if( (posX < xMinThres) | (posX > xMaxThres) | (posY < yMinThres) | (posY > yMaxThres) ) {//try to centered on the screen
			if(posX < xMinThres) 		ROS_INFO("Moving left");//message.camera = 'p';
			else if(posX > xMaxThres) 	ROS_INFO("Moving right");//message.camera = 'm';
			
			//if( posY < yMinThres) 	/*ROS_INFO("Moving Forward");*/message.motor = 'f';
			//else if(posY> yMaxThres) /*ROS_INFO("Moving backwards");*/message.motor = 'r';
			
			message.motor = 's'; 			
			message.errorOpenCV = '3';
			opencvCommands.publish( message );
		    }
		    else {
			message.camera = 'k'; 
			message.motor = 's'; 
			message.errorOpenCV = '3';
			opencvCommands.publish(message);
			// ROS_INFO("Target Acquired");
		    }
  
}

void sendError(){
	message.camera = 's';
	message.motor = 's';
	message.errorOpenCV = '1';
	opencvCommands.publish( message );
	
	exit(1);
}
