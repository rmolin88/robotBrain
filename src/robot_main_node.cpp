#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
//#include <robotBrain/opencvMessage.h>
#include <SerialStream.h>
#include <math.h>

#define SONAR_THRESHOLD 1300
#define CLOSE_SONAR_THRESHOLD 1050
#define MAIN_LOOP_RATE 10
#define RC_LOOP_RATE 20
#define PAN_VALUE 80
#define MOTOR_OFF_TIME 10
#define ERROR_FREE '0'
#define ERROR '1'
#define RC_COMMAND '2'
#define TARGET_FOUND '3'

class robot{
	public:
	//ros variables
	ros::NodeHandle nh_; 
//	ros::Subscriber opencvCommands;
	ros::Subscriber joy_subscriber_;
	ros::Rate main_loop_rate_ ( MAIN_LOOP_RATE );
    ros::Rate rc_loop_rate_ ( RC_LOOP_RATE );

	//remote control variables
    float subscriber_forward_throttle_; 	
	float subscriber_reverse_throttle_;

	int32_t subscriber_right_steer_;
	int32_t subscriber_left_steer_;
	int32_t subscriber_buttonA_;		//buttons messages are ints
	int32_t subscriber_buttonB_;
	
	//program variables
    uint8_t remote_control_;
	uint8_t opencv_;

	uint16_t left_sonar_;		//left sonar
	uint16_t center_sonar_;		//center
	uint16_t right_sonar_;		//right
	
	char motor_temp_;
    uint8_t motor_timeout_;
    bool motor_pause_;
    

	char motor_;
	char servo_;
	char camera_steering_;
	char error_xmega_;		//0 means everything ok - 1 means error or closed application
	char error_OpenCV_;

    
	
	robot(){
		//opencvCommands = n.subscribe<robotBrain::opencvMessage> ( "opencv_commands", 1000, opencvCallback );
		joy_subscriber_ = nh_.subscribe<sensor_msgs::Joy>("joy",1000, &robot::joy_callback, this);
		connect_to_serial();
	}
	
	void robot::joy_callback(const sensor_msgs::Joy::ConstPtr& joy);
	//void robot::opencvCallback (const robotBrain::opencvMessage::ConstPtr& opencvMessage);
	void robot::do_obs_avoidance ( char read[] );
	void robot::do_remote_control();
	void robot::connect_to_serial();
	void robot::serial_transmit(char motor, char servo, char camera_steering, char error_xmega_);
	void robot::pan_camera_servo();
};

int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "robot_main_node" );

    robot Sparky;
    
    //initializing variables
    remote_control_ = false;
    opencv_ = false;
    motor_pause_ = false;
    
    motor_temp_ = 'f';
    motor_ = 'f';
    servo_ = 's';
    camera_steering_ = 's';
    error_xmega_ = ERROR_FREE;
    error_OpenCV_ = ERROR_FREE;
    motor_timeout_ = MOTOR_OFF_TIME;
    
    char read[13];
    char cameraTemp = 'p';
    uint8_t pcounter = PAN_VALUE;
    
    
    while ( ( mySerial.good() ) & ( n.ok() ) ) {
      
		if( !(error_OpenCV_ == TARGET_FOUND) ) Sparky.pan_camera_servo();
		
		Sparky.serial_transmit(motor_, servo_, camera_steering_, error_xmega_);
		  
		ROS_INFO("[%c %c %c] Commands to Atxmega ", motor_, servo_, camera_steering_);
		  
		if ( mySerial.rdbuf()->in_avail() ) mySerial.read ( read, sizeof ( read ) );

		//ROS_INFO("%s Sonars", read);
		
		
		
		ros::spinOnce();  //spinning here to get values from remote and/or opencv

		while ( remote_control_ ) {

			Sparky.do_remote_control();
				
			ROS_INFO("[%c %c] Teleop Commands", motor_, servo_);
			
			ros::spinOnce();
			
			rc_loop_rate_.sleep();
		}

		if( (!(robot::motor_ == 's') && ( (opencv_) || (motor_pause_) ) Sparky.do_obs_avoidance ( read ); //do not avoid obstacles if motors are stop
		main_loop_rate_.sleep();
    }

	motor_ = servo_ = camera_steering_ = 's';
	error_xmega_ = ERROR;
	ROS_FATAL ( "ROS/COMMUNICATION FAILURE CLOSING NODE" );
	Sparky.serial_transmit(motor_, servo_, camera_steering_, error_xmega_)

    return -1;
}
    

void robot::do_remote_control() {

	  if(subscriber_right_steer_) servo_ = 'r'; 			
	  else if(subscriber_left_steer_)servo_ = 'l';			
	  else servo_ = 's'; 			

	  if(subscriber_forward_throttle_< 0) motor_ = 'f'; 		//we move faster when doing remote control
	  else if(subscriber_reverse_throttle_ < 0) motor_ = 'r';
	  else  motor_ = 's'; 

}

void robot::do_obs_avoidance ( char read[] ) {
  
  left_sonar_ = 0;		//left sonar
  center_sonar_ = 0;		//center
  right_sonar_ = 0;	
  
  if ( !(motor_pause_) ) motor_temp_ = motor_;	//changing motorTemp only if motor is not paused
  
  //converting char strings to ints
  for ( int i = 0; i<4; i++ ) {
        switch ( i ) {
            case 0:
                left_sonar_ 	+= ( read[0] - '0' ) *1000;
                center_sonar_ 	+= ( read[4] - '0' ) *1000;
                right_sonar_ 	+= ( read[8] - '0' ) *1000;

                break;
            case 1:
                left_sonar_ 	+= ( read[1] - '0' ) *100;
                center_sonar_ 	+= ( read[5] - '0' ) *100;
                right_sonar_ 	+= ( read[9] - '0' ) *100;

                break;
            case 2:
                left_sonar_ 	+= ( read[2] - '0' ) *10;
                center_sonar_ 	+= ( read[6] - '0' ) *10;
                right_sonar_ 	+= ( read[10] -'0' ) *10;

                break;
            case 3:
                left_sonar_ 	+= ( read[3] - '0' );
                center_sonar_ 	+= ( read[7] - '0' );
                right_sonar_ 	+= ( read[11] -'0' );

                break;
            }
        }
        ROS_INFO("[%d %d %d] Left Center Right Sonar Values", left_sonar_, center_sonar_, right_sonar_);
        
    if ( ( left_sonar_ < CLOSE_SONAR_THRESHOLD ) || ( center_sonar_ < CLOSE_SONAR_THRESHOLD ) || ( right_sonar_ < CLOSE_SONAR_THRESHOLD ) ) {
		motor_ = 'r';
		return;
    }
    else if ( ( left_sonar_ < SONAR_THRESHOLD ) || ( center_sonar_ < SONAR_THRESHOLD ) || ( right_sonar_ < SONAR_THRESHOLD ) ) {
	
		if (right_sonar_ < SONAR_THRESHOLD) right_sonar_ = 1;
		else right_sonar_ = 0;
		
		if (left_sonar_ < SONAR_THRESHOLD) left_sonar_= 1;
		else left_sonar_ = 0;
		
		if (center_sonar_ < SONAR_THRESHOLD) center_sonar_ = 1;
		else center_sonar_ = 0;
	}
	
	//straight
	if( ((!left_sonar_) && (!center_sonar_) && (!right_sonar_)) || ((left_sonar_) && (!center_sonar_) && (right_sonar_)) ){
	  servo_ = 's';
	  motor_ = 'f';
	}
	
	//left
	else if ( (right_sonar_ && (!left_sonar_)) || ((!left_sonar_) & center_sonar_) ){
	  servo_ = 'l';
	  motor_ = 'f';
	  
	}
	
	//right
	else if( (!right_sonar_) && left_sonar_ ){
	  servo_ = 'r';
	  motor_ = 'f';
	}
	
	//reverse
	else {
	  servo_ = 's';
	  motor_ = 'r';
	  
	}
	 //break if there is a change in direction
	if( /*(motor_ == 'f' & motor_temp_ == 'r') || */ (motor_ == 'r' & motor_temp_ =='f') || (motor_pause_) ) {
	  motor_pause_ = true;
	  motor_ = servo_ = 's';
	  motor_timeout_--;
	  if(!motor_timeout_) {motor_pause_ = false; motor_timeout_ = 10;}	  
	}
	
	
}
   


//~ void robot::opencvCallback (const robotBrain::opencvMessage::ConstPtr& opencvMessage){
    //~ //ROS_INFO("TARGET ACQUIRED");
    //~ cameraSteering = 	(char) (opencvMessage->camera);		
    //~ motor = 		(char) (opencvMessage->motor);
    //~ errorOpenCV = 	(char) (opencvMessage -> errorOpenCV);
    //~ if( !remote_control_ ) opencv_ = true;
    //~ errorXmega = errorOpenCV;
//~ }

void robot::joy_callback(const sensor_msgs::Joy::ConstPtr& joy){
	subscriber_reverse_throttle_ 	= joy->axes[2] ;		
	subscriber_forward_throttle_	= joy->axes[5];
	subscriber_right_steer_ 		= joy->buttons[11] ;		
	subscriber_left_steer_ 			= joy->buttons[12] ;
	subscriber_buttonA_ 			= joy->buttons[0];
	subscriber_buttonB_ 			= joy->buttons[1];
	
	if ( subscriber_buttonA_ ) {remote_control_ = true; opencv_ = false; error_xmega_ = RC_COMMAND;camera_steering_ = 's';}
    if ( subscriber_buttonB_ ) {remote_control_ = false; error_xmega_ = ERROR_FREE; }
 }


void robot::connect_to_serial(){
	//SERIAL INIT

    LibSerial::SerialStream mySerial;

    mySerial.Open ( "/dev/ttyUSB0" );

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


void robot::serial_transmit(char motor, char servo, char camera_steering, char error_xmega){
	mySerial.write(&motor, 1);
	mySerial.write(&servo, 1);
	mySerial.write(&camera_steering, 1);
	mySerial.write(&error_xmega, 1);
}

void robot::pan_camera_servo(){
	camera_steering_ = cameraTemp;
	pcounter--;
	if (pcounter == PAN_VALUE/2) cameraTemp = 'm';
	else if(!pcounter) {pcounter = PAN_VALUE; cameraTemp = 'p';}
}
