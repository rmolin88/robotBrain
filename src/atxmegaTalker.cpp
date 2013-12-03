#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <robotBrain/opencvMessage.h>
#include <SerialStream.h>
#include <math.h>
#include <iostream>


#define sonarThreshold 1300
#define verycloseThreshold 1050
#define loopRate 10
#define remoteRate 20

/**************************************************************************************
*Node Objectives
*	Receives Sonar Feedback from the Atxmega Board through Serial Communication
*	Pruocess the Information and does obstacle Avoidance
*	Controls Motor and Servo
*	If the Joy Node is transmitting takes control over the motor and servo
*	Sends feedback to Station(Computer on the same network running the Joy Node)
*
****************************************************************************************/

using namespace LibSerial;
using namespace ros;
using namespace std;
        
void obstacleAvoidance ( char read[] );
void remoteControlCommands();

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
void opencvCallback (const robotBrain::opencvMessage::ConstPtr& opencvMessage);

Subscriber opencvCommands;
Subscriber joy_subscriber;


bool remoteControl = false;
bool opencv = false;

float subscriberSteer;			//axis messages are floats
float subscriberThrottle;
float subscriberForwardThrottle; 	
float subscriberReverseThrottle;
	
int32_t subscriberButtonA;		//buttons messages are ints
int32_t subscriberButtonB;
int32_t subscriberButtonY;

int16_t leftSonar = 0;		//left sonar
int16_t centerSonar = 0;		//center
int16_t rightSonar = 0;		//right

uint8_t Straight;
uint8_t Left;
uint8_t Right;
uint8_t Reverse;
    
char motor = 'f';
char servo = 's';
char cameraSteering = 's';
char errorXmega = '0';		//0 means everything ok - 1 means error or closed application
char errorOpenCV = '0';

int main ( int argc, char **argv ) {

    //ROS INIT

    init ( argc, argv, "atxmegaTalker" );

    NodeHandle n;

    opencvCommands = n.subscribe<robotBrain::opencvMessage> ( "opencv_commands", 1000, opencvCallback );
    joy_subscriber = n.subscribe<sensor_msgs::Joy>("joy",1000, joyCallback);
  
    Rate loop_rate ( loopRate );
    Rate remoteControlLoopRate ( remoteRate );
    
    //SERIAL INIT

    SerialStream mySerial;

    mySerial.Open ( "/dev/ttyUSB0" );

    mySerial.SetBaudRate ( SerialStreamBuf::BAUD_57600 );

    mySerial.SetCharSize ( SerialStreamBuf::CHAR_SIZE_8 );

    mySerial.SetNumOfStopBits ( 1 );

    mySerial.SetParity ( SerialStreamBuf::PARITY_NONE );

    mySerial.SetFlowControl ( SerialStreamBuf::FLOW_CONTROL_NONE );

    if ( !mySerial.good() ) {
        ROS_FATAL ( "COULD NOT SET UP SERIAL COMMUNICATION CLOSING NODE" );
        exit ( 1 );
        }
	  
    char read[13];
    char motorTemp = 0;
    uint8_t changeInDirectionCounter = 0;
    bool motorPause = false;
    
    while ( ( mySerial.good() ) & ( n.ok() ) ) {
      
      
	if( (motor == 'f' & motorTemp == 'r') | (motor == 'r' & motorTemp =='f') | (motorPause) ) {
	  motorPause = true;
	  motor = 's';
	  servo = 's';
	  changeInDirectionCounter++;
	  if(changeInDirectionCounter == 10) {motorPause = false; changeInDirectionCounter = 0;}	  
	}
      
	ROS_INFO("[%c %c %c] Commands to Atxmega ", motor, servo, cameraSteering);
	  
	mySerial.write(&motor, 1);
	mySerial.write(&servo, 1);
	mySerial.write(&cameraSteering, 1);
	mySerial.write(&errorXmega, 1);

	if ( mySerial.rdbuf()->in_avail() ) mySerial.read ( read, sizeof ( read ) );

	//ROS_INFO("%s Sonars", read);
	
	if ( !(motorPause) ) motorTemp = motor;	//changing motorTemp only if motor is not paused
	
	spinOnce();  //spinning here to get values from remote and/or opencv

	while ( remoteControl ) {

		spinOnce();
		remoteControlCommands();
			
		ROS_INFO("[%c %c Teleop Commands]", motor, servo);
		
		mySerial.write(&motor, 1);
		mySerial.write(&servo, 1);
		mySerial.write(&cameraSteering, 1);
		mySerial.write(&errorXmega, 1);
		
		remoteControlLoopRate.sleep();
	}

	if( (!(motor == 's') & (opencv)) | (!opencv) ) obstacleAvoidance ( read ); //do not avoid obstacles if motors are stop by opencv
	loop_rate.sleep();
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
    

void remoteControlCommands() {

	  if(subscriberSteer > 0) servo = 'r'; 			
	  else if(subscriberSteer < 0)servo = 'l';			
	  else if(!subscriberSteer) servo = 's'; 			

	  if(subscriberForwardThrottle< 0) motor = 'F'; 		//we move faster when doing remote control
	  else if(subscriberReverseThrottle < 0) motor = 'R';
	  else  motor = 's'; 

	  cameraSteering = 's';
	
}

void obstacleAvoidance ( char read[] ) {
  
  leftSonar = 0;		//left sonar
  centerSonar = 0;		//center
  rightSonar = 0;	
  
  for ( int i = 0; i<4; i++ ) {
        switch ( i ) {
            case 0:
                leftSonar += ( read[0] - '0' ) *1000;
                centerSonar += ( read[4] - '0' ) *1000;
                rightSonar += ( read[8] - '0' ) *1000;

                break;
            case 1:
                leftSonar += ( read[1] - '0' ) *100;
                centerSonar += ( read[5] - '0' ) *100;
                rightSonar += ( read[9] - '0' ) *100;

                break;
            case 2:
                leftSonar += ( read[2] - '0' ) *10;
                centerSonar += ( read[6] - '0' ) *10;
                rightSonar += ( read[10] - '0' ) *10;

                break;
            case 3:
                leftSonar += ( read[3] - '0' );
                centerSonar += ( read[7] - '0' );
                rightSonar += ( read[11] - '0' );

                break;
            }
        }
        ROS_INFO("[%d %d %d] Left Center Right Sonar Values", leftSonar, centerSonar, rightSonar);
        
    if ( ( leftSonar < verycloseThreshold ) | ( centerSonar < verycloseThreshold ) | ( rightSonar < verycloseThreshold ) ) {
	motor = 'r';
	//servo = 's';
    }
    else if ( ( leftSonar < sonarThreshold ) | ( centerSonar < sonarThreshold ) | ( rightSonar < sonarThreshold ) ) {
	if (rightSonar < sonarThreshold) rightSonar = 1;
	else rightSonar = 0;
	
	if (leftSonar < sonarThreshold) leftSonar= 1;
	else leftSonar= 0;
	
	if (centerSonar < sonarThreshold) centerSonar = 1;
	else centerSonar = 0;
	
	Straight =  ( ((1-leftSonar) & (1-centerSonar) & (1-rightSonar)) | ((leftSonar) & (1-centerSonar) & (rightSonar)) );
	if(Straight){
	  servo = 's';
	  motor = 'f';
	  return;
	}
	
	Left = ( (rightSonar & (1-leftSonar)) | ((1-leftSonar) & centerSonar) );
	if(Left){
	  servo = 'l';
	  motor = 'f';
	  return;
	}
	
	Right = ( (1-rightSonar) & leftSonar );
	if(Right){
	  servo = 'r';
	  motor = 'f';
	  return;
	}
	
	Reverse = ( (leftSonar & centerSonar & rightSonar) );//| ((1-leftSonar) & (centerSonar) & (1-rightSonar)) );
	if(Reverse){
	  servo = 's';
	  motor = 'r';
	  return;
	}
    }
    else{ //if there is no obstacle in front we turn towards object if theres any detected
	if( (cameraSteering == 'p')  ) servo = 'r'; 	
	else if( (cameraSteering == 'm') ) servo = 'l';
	else servo = 's';
	
	if(!opencv) motor = 'f'; //if receiving commands from camera dont change them
    }
}


void opencvCallback (const robotBrain::opencvMessage::ConstPtr& opencvMessage){
    //ROS_INFO("TARGET ACQUIRED");
    cameraSteering = 	(char) (opencvMessage->camera);		
    motor = 		(char) (opencvMessage->motor);
    errorOpenCV = 	(char) (opencvMessage -> errorOpenCV);
    if( !remoteControl ) opencv = true;
    errorXmega = errorOpenCV;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
	//axes[0] is the left joystick left/right
	//axes[2] LT
	//axes[5] RT
	
	subscriberReverseThrottle 	=  joy->axes[2] ;		//ranging from -100 to 100
	subscriberForwardThrottle	= joy->axes[5];
	subscriberSteer 	= ( joy->axes[6] );		//-30 to 30
	subscriberButtonA 	= joy->buttons[0];
	subscriberButtonB 	= joy->buttons[1];
	subscriberButtonY	= joy->buttons[3];
    if ( subscriberButtonA ) {remoteControl = true; opencv = false; errorXmega = '2';}
   if ( subscriberButtonB ) {remoteControl = false; errorXmega = '0'; }
  
}
