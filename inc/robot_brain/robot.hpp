#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <SerialStream.h>
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include "robot_brain/opencv.h"

#define SONAR_THRESHOLD 1300
#define CLOSE_SONAR_THRESHOLD 1050
#define MAIN_LOOP_RATE 10
#define RC_LOOP_RATE 15
#define PAN_VALUE 80
#define MOTOR_OFF_TIME 10
#define ERROR_FREE '0'
#define ERROR '1'
#define RC_COMMAND '2'
#define TARGET_FOUND '3'
#define SERIAL_RECEIVE_SIZE 13

class robot{
	public:
	//ros variables
	ros::NodeHandle nh_; 
	ros::Subscriber joy_subscriber_;
	ros::Subscriber opencv_sub_;
	ros::Rate main_loop_rate_;
    ros::Rate rc_loop_rate_;
    
	//remote control variables
    float subscriber_forward_throttle_; 	
	float subscriber_reverse_throttle_;

	int32_t subscriber_right_steer_;
	int32_t subscriber_left_steer_;
	int32_t subscriber_buttonA_;
	int32_t subscriber_buttonB_;
	
	//program variables
    uint8_t remote_control_;
	uint8_t opencv_;

	uint16_t left_sonar_;	
	uint16_t center_sonar_;	
	uint16_t right_sonar_;	
	
	char motor_temp_;
    uint8_t motor_timeout_;
    bool motor_pause_;
    

	char motor_;
	char servo_;
	char camera_steering_;
	char feedback_xmega_;		
	char OpenCV_feedback_;
	
	char camera_temp_;
	uint8_t pan_counter_;
	char serial_read_[SERIAL_RECEIVE_SIZE];
	
	LibSerial::SerialStream mySerial;

	robot(std::string serial_name):
				remote_control_(false),
				opencv_(false),
				motor_pause_(false),
				
				motor_temp_('f'),
				motor_('f'),
				servo_('s'),
				camera_steering_('s'),
				feedback_xmega_(ERROR_FREE),
				OpenCV_feedback_(ERROR_FREE),
				motor_timeout_(MOTOR_OFF_TIME),
				
				camera_temp_('p'),
				pan_counter_(PAN_VALUE),
				main_loop_rate_(10),
				rc_loop_rate_(15)
				
	{
		opencv_sub_ = nh_.subscribe<robot_brain::opencv> ( "opencv_commands", 1000, &robot::opencv_callback, this );
		joy_subscriber_ = nh_.subscribe<sensor_msgs::Joy>("joy",1000, &robot::joy_callback, this);
		
		mySerial.Open ( serial_name );

		mySerial.SetBaudRate ( LibSerial::SerialStreamBuf::BAUD_57600 );

		mySerial.SetCharSize ( LibSerial::SerialStreamBuf::CHAR_SIZE_8 );

		mySerial.SetNumOfStopBits ( 1 );

		mySerial.SetParity ( LibSerial::SerialStreamBuf::PARITY_NONE );

		mySerial.SetFlowControl ( LibSerial::SerialStreamBuf::FLOW_CONTROL_NONE );
		
		if ( !mySerial.good() ) {
		ROS_FATAL("--(!)Failed to Open Serial Port(!)--");
		exit(-1);
		}
	}
	
	void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);
	void opencv_callback (const robot_brain::opencv::ConstPtr& opencv_msg);
	void do_obs_avoidance ( char serial_read_[] );
	void do_remote_control();
	void pan_camera_servo();
	void transmit_to_serial(char motor, char servo, char camera_steering, char feedback_xmega_);
};

/*class serial{
	public:
	LibSerial::SerialStream mySerial;

	serial(std::string serial_name){
		
		mySerial.Open ( serial_name );

		mySerial.SetBaudRate ( LibSerial::SerialStreamBuf::BAUD_57600 );

		mySerial.SetCharSize ( LibSerial::SerialStreamBuf::CHAR_SIZE_8 );

		mySerial.SetNumOfStopBits ( 1 );

		mySerial.SetParity ( LibSerial::SerialStreamBuf::PARITY_NONE );

		mySerial.SetFlowControl ( LibSerial::SerialStreamBuf::FLOW_CONTROL_NONE );
		
		if ( !mySerial.good() ) {
		ROS_FATAL("--(!)Failed to Open Serial Port(!)--");
		exit(-1);
		}
	}
	
	void transmit_to_serial(char motor, char servo, char camera_steering, char feedback_xmega_);
};
*/
#endif
