#include <math.h>
#include <robot_brain/robot.hpp>

int main ( int argc, char **argv ) {
	
	
    ros::init ( argc, argv, "robot_main_node" );
    ros::Rate main_loop_rate_(MAIN_LOOP_RATE);
    ros::Rate rc_loop_rate_(RC_LOOP_RATE);
    
    //Serial Init
    serial xmega_serial("/dev/ttyUSB0");
	
	//creating robot :)
    robot Sparky;
    
    //initializing variables
    Sparky.remote_control_ = false;
    Sparky.opencv_ = false;
    Sparky.motor_pause_ = false;
    
    Sparky.motor_temp_ = 'f';
    Sparky.motor_ = 'f';
    Sparky.servo_ = 's';
    Sparky.camera_steering_ = 's';
    Sparky.feedback_xmega_ = ERROR_FREE;
    Sparky.OpenCV_feedback_ = ERROR_FREE;
    Sparky.motor_timeout_ = MOTOR_OFF_TIME;
    
    Sparky.camera_temp_ = 'p';
    Sparky.pan_counter_ = PAN_VALUE;
    
    Sparky.serial_read_[SERIAL_RECEIVE_SIZE] = 0;
    
    while ( ( xmega_serial.mySerial.good() ) & ( Sparky.nh_.ok() ) ) {
      
		if( !(Sparky.OpenCV_feedback_ == TARGET_FOUND) ) Sparky.pan_camera_servo();
		
		xmega_serial.transmit_to_serial(Sparky.motor_, Sparky.servo_, Sparky.camera_steering_, Sparky.feedback_xmega_);
		  
		ROS_INFO("[%c %c %c] Commands to Atxmega ", Sparky.motor_, Sparky.servo_, Sparky.camera_steering_);
		  
		if ( xmega_serial.mySerial.rdbuf()->in_avail() ) xmega_serial.mySerial.read ( Sparky.serial_read_, SERIAL_RECEIVE_SIZE );
		
		ros::spinOnce();  //spinning here to get values from remote and opencv

		while ( Sparky.remote_control_ ) {

			Sparky.do_remote_control();
				
			ROS_INFO("[%c %c] Teleop Commands", Sparky.motor_, Sparky.servo_);
			
			ros::spinOnce();
			
			rc_loop_rate_.sleep();
		}

		if( !((Sparky.motor_ == 's') && !(Sparky.motor_pause_)) ) Sparky.do_obs_avoidance ( Sparky.serial_read_ ); //do not avoid obstacles if motors are stop
		
		main_loop_rate_.sleep();
    }

	Sparky.motor_ = Sparky.servo_ = Sparky.camera_steering_ = 's';
	Sparky.feedback_xmega_ = ERROR;
	ROS_FATAL ( "ROS/COMMUNICATION FAILURE CLOSING NODE" );
	xmega_serial.transmit_to_serial(Sparky.motor_, Sparky.servo_, Sparky.camera_steering_, Sparky.feedback_xmega_);

    return -1;
}
