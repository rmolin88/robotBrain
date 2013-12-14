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
    uint8_t remote_control_ = false;
	uint8_t opencv_ = false;

	uint16_t left_sonar_;		//left sonar
	uint16_t center_sonar_;		//center
	uint16_t right_sonar_;		//right

	char motor = 'f';
	char servo = 's';
	char cameraSteering = 's';
	char errorXmega = '0';		//0 means everything ok - 1 means error or closed application
	char errorOpenCV = '0';

    
	
	robot(){
		//opencvCommands = n.subscribe<robotBrain::opencvMessage> ( "opencv_commands", 1000, opencvCallback );
		joy_subscriber_ = nh_.subscribe<sensor_msgs::Joy>("joy",1000, joyCallback);
		connect_to_serial();
		
	}
	
	void robot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	//void robot::opencvCallback (const robotBrain::opencvMessage::ConstPtr& opencvMessage);
	void robot::do_obs_avoidance ( char read[] );
	void robot::remote_control_Commands();
	void robot::connect_to_serial();
};









/**************************************************************************************
*Node Objectives
*	Receives Sonar Feedback from the Atxmega Board through Serial Communication
*	Pruocess the Information and does obstacle Avoidance
*	Controls Motor and Servo
*	If the Joy Node is transmitting takes control over the motor and servo
*	Sends feedback to Station(Computer on the same network running the Joy Node)
*
****************************************************************************************/

        






int main ( int argc, char **argv ) {

    //ROS INIT

    ros::init ( argc, argv, "robot_main_node" );

    robot 
    
    
    
	  
    char read[13];
    char motorTemp = 'f';
    uint8_t changeInDirectionCounter = 10;
    bool motorPause = false;
    char cameraTemp = 'p';
    uint8_t pcounter = PAN_VALUE;
    
    
    while ( ( mySerial.good() ) & ( n.ok() ) ) {
      
      //break if there is a change in direction
	if( (motor == 'f' & motorTemp == 'r') | (motor == 'r' & motorTemp =='f') | (motorPause) ) {
	  motorPause = true;
	  motor = servo = 's';
	  changeInDirectionCounter--;
	  if(!changeInDirectionCounter) {motorPause = false; changeInDirectionCounter = 10;}	  
	}
	
	if( !(errorOpenCV == '3') ) {	//if camera has not found target pan left to right
	    cameraSteering = cameraTemp;
	    pcounter--;
	    if (pcounter == PAN_VALUE/2) cameraTemp = 'm';
	    else if(!pcounter) {pcounter = PAN_VALUE; cameraTemp = 'p';}
	}
      
	ROS_INFO("[%c %c %c] Commands to Atxmega ", motor, servo, cameraSteering);
	  
	mySerial.write(&motor, 1);
	mySerial.write(&servo, 1);
	mySerial.write(&cameraSteering, 1);
	mySerial.write(&errorXmega, 1);

	if ( mySerial.rdbuf()->in_avail() ) mySerial.read ( read, sizeof ( read ) );

	//ROS_INFO("%s Sonars", read);
	
	if ( !(motorPause) ) motorTemp = motor;	//changing motorTemp only if motor is not paused
	
	ros::spinOnce();  //spinning here to get values from remote and/or opencv

	while ( remote_control_ ) {

		remote_control_Commands();
			
		ROS_INFO("[%c %c] Teleop Commands", motor, servo);
		
		mySerial.write(&motor, 1);
		mySerial.write(&servo, 1);
		mySerial.write(&cameraSteering, 1);
		mySerial.write(&errorXmega, 1);
		ros::spinOnce();
		
		rc_loop_rate_.sleep();
		//reason why it might be breaking not expecting the xmega error command on xmega board
	}

	if( (!(motor == 's') & (opencv_)) | (!opencv_) ) do_obs_avoidance ( read ); //do not avoid obstacles if motors are stop by opencv
	main_loop_rate_.sleep();
    }

	motor = servo = cameraSteering = 's';
	errorXmega = '1';
	ROS_FATAL ( "ROS/COMMUNICATION FAILURE CLOSING NODE" );
	mySerial.write(&motor, 1);
	mySerial.write(&servo, 1);
	mySerial.write(&cameraSteering, 1);
	mySerial.write(&errorXmega, 1);

    return -1;
}
    

void robot::remote_control_Commands() {

	  if(subscriber_right_steer_) servo = 'r'; 			
	  else if(subscriber_left_steer_)servo = 'l';			
	  else servo = 's'; 			

	  if(subscriber_forward_throttle_< 0) motor = 'f'; 		//we move faster when doing remote control
	  else if(subscriber_reverse_throttle_ < 0) motor = 'r';
	  else  motor = 's'; 

}

void robot::do_obs_avoidance ( char read[] ) {
  
  left_sonar_ = 0;		//left sonar
  center_sonar_ = 0;		//center
  right_sonar_ = 0;	
  
  //converting char strings to ints
  for ( int i = 0; i<4; i++ ) {
        switch ( i ) {
            case 0:
                left_sonar_ += ( read[0] - '0' ) *1000;
                center_sonar_ += ( read[4] - '0' ) *1000;
                right_sonar_ += ( read[8] - '0' ) *1000;

                break;
            case 1:
                left_sonar_ += ( read[1] - '0' ) *100;
                center_sonar_ += ( read[5] - '0' ) *100;
                right_sonar_ += ( read[9] - '0' ) *100;

                break;
            case 2:
                left_sonar_ += ( read[2] - '0' ) *10;
                center_sonar_ += ( read[6] - '0' ) *10;
                right_sonar_ += ( read[10] - '0' ) *10;

                break;
            case 3:
                left_sonar_ += ( read[3] - '0' );
                center_sonar_ += ( read[7] - '0' );
                right_sonar_ += ( read[11] - '0' );

                break;
            }
        }
        ROS_INFO("[%d %d %d] Left Center Right Sonar Values", left_sonar_, center_sonar_, right_sonar_);
        
    if ( ( left_sonar_ < CLOSE_SONAR_THRESHOLD ) | ( center_sonar_ < CLOSE_SONAR_THRESHOLD ) | ( right_sonar_ < CLOSE_SONAR_THRESHOLD ) ) {
		motor = 'r';
		return;
    }
    else if ( ( left_sonar_ < SONAR_THRESHOLD ) | ( center_sonar_ < SONAR_THRESHOLD ) | ( right_sonar_ < SONAR_THRESHOLD ) ) {
	
		if (right_sonar_ < SONAR_THRESHOLD) right_sonar_ = 1;
		else right_sonar_ = 0;
		
		if (left_sonar_ < SONAR_THRESHOLD) left_sonar_= 1;
		else left_sonar_ = 0;
		
		if (center_sonar_ < SONAR_THRESHOLD) center_sonar_ = 1;
		else center_sonar_ = 0;
	}
	
	//straight
	if( ((!left_sonar_) & (!center_sonar_) & (!right_sonar_)) | ((left_sonar_) & (!center_sonar_) & (right_sonar_)) ){
	  servo = 's';
	  motor = 'f';
	}
	
	//left
	else if ( (right_sonar_ & (!left_sonar_)) | ((!left_sonar_) & center_sonar_) ){
	  servo = 'l';
	  motor = 'f';
	  
	}
	
	//right
	else if( (!right_sonar_) & left_sonar_ ){
	  servo = 'r';
	  motor = 'f';
	}
	
	//reverse
	else {
	  servo = 's';
	  motor = 'r';
	  
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

void robot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
	subscriber_reverse_throttle_ 	=  joy->axes[2] ;		
	subscriber_forward_throttle_	= joy->axes[5];
	subscriber_right_steer_ 		= joy->buttons[11] ;		
	subscriber_left_steer_ 		= joy->buttons[12] ;
	subscriber_buttonA_ 		= joy->buttons[0];
	subscriber_buttonB_ 		= joy->buttons[1];
	
	if ( subscriber_buttonA_ ) {remote_control_ = true; opencv_ = false; errorXmega = '2';cameraSteering = 's';}
    if ( subscriber_buttonB_ ) {remote_control_ = false; errorXmega = '0'; }
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
