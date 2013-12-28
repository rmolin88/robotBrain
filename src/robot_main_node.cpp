#include <math.h>
#include <robot_brain/robot.hpp>

int main ( int argc, char **argv ) {
	
	ros::init ( argc, argv, "robot_main_node" );
    
	//creating robot :)
    robot Sparky("/dev/ttyUSB0");
    
    //Serial Init
    //serial xmega_serial("/dev/ttyUSB0");
	
    //ros::Rate main_loop_rate_(MAIN_LOOP_RATE);
    //ros::Rate rc_loop_rate_(RC_LOOP_RATE);
    
    
    while ( ( Sparky.mySerial.good() ) & ( Sparky.nh_.ok() ) ) {
      
		if( !(Sparky.OpenCV_feedback_ == TARGET_FOUND) ) Sparky.pan_camera_servo();
		
		Sparky.transmit_to_serial(Sparky.motor_, Sparky.servo_, Sparky.camera_steering_, Sparky.feedback_xmega_);
		  
		ROS_INFO("[%c %c %c] Commands to Atxmega ", Sparky.motor_, Sparky.servo_, Sparky.camera_steering_);
		  
		if ( Sparky.mySerial.rdbuf()->in_avail() ) Sparky.mySerial.read ( Sparky.serial_read_, SERIAL_RECEIVE_SIZE );
		
		ros::spinOnce();  //spinning here to get values from remote and opencv

		while ( Sparky.remote_control_ ) {

			Sparky.do_remote_control();
				
			ROS_INFO("[%c %c] Teleop Commands", Sparky.motor_, Sparky.servo_);
			
			ros::spinOnce();
			
			Sparky.transmit_to_serial(Sparky.motor_, Sparky.servo_, Sparky.camera_steering_, Sparky.feedback_xmega_);
			
			Sparky.rc_loop_rate_.sleep();
		}

		if( !((Sparky.motor_ == 's') && !(Sparky.motor_pause_)) ) Sparky.do_obs_avoidance ( Sparky.serial_read_ ); //do not avoid obstacles if motors are stop
		
		Sparky.main_loop_rate_.sleep();
    }

	Sparky.motor_ = Sparky.servo_ = Sparky.camera_steering_ = 's';
	Sparky.feedback_xmega_ = ERROR;
	ROS_FATAL ( "ROS/COMMUNICATION FAILURE CLOSING NODE" );
	Sparky.transmit_to_serial(Sparky.motor_, Sparky.servo_, Sparky.camera_steering_, Sparky.feedback_xmega_);

    return -1;
}
