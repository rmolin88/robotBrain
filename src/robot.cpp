#include <robot_brain/robot.hpp>

void robot::do_remote_control() {

	  if(subscriber_right_steer_) servo_ = 'r'; 			
	  else if(subscriber_left_steer_)servo_ = 'l';			
	  else servo_ = 's'; 			

	  if(subscriber_forward_throttle_< 0) motor_ = 'f'; 		//we move faster when doing remote control
	  else if(subscriber_reverse_throttle_ < 0) motor_ = 'r';
	  else  motor_ = 's'; 
}

void robot::do_obs_avoidance ( char serial_read_[] ) {
  
  left_sonar_ = 0;		//left sonar
  center_sonar_ = 0;		//center
  right_sonar_ = 0;	
  
  if ( !(motor_pause_) ) motor_temp_ = motor_;	//changing motorTemp only if motor is not paused
  
  //converting char strings to ints
  for ( int i = 0; i<4; i++ ) {
        switch ( i ) {
            case 0:
                left_sonar_ 	+= ( serial_read_[0] - '0' ) *1000;
                center_sonar_ 	+= ( serial_read_[4] - '0' ) *1000;
                right_sonar_ 	+= ( serial_read_[8] - '0' ) *1000;

                break;
            case 1:
                left_sonar_ 	+= ( serial_read_[1] - '0' ) *100;
                center_sonar_ 	+= ( serial_read_[5] - '0' ) *100;
                right_sonar_ 	+= ( serial_read_[9] - '0' ) *100;

                break;
            case 2:
                left_sonar_ 	+= ( serial_read_[2] - '0' ) *10;
                center_sonar_ 	+= ( serial_read_[6] - '0' ) *10;
                right_sonar_ 	+= ( serial_read_[10] -'0' ) *10;

                break;
            case 3:
                left_sonar_ 	+= ( serial_read_[3] - '0' );
                center_sonar_ 	+= ( serial_read_[7] - '0' );
                right_sonar_ 	+= ( serial_read_[11] -'0' );

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
   

void robot::joy_callback(const sensor_msgs::Joy::ConstPtr& joy){
	subscriber_reverse_throttle_ 	= joy->axes[2] ;		
	subscriber_forward_throttle_	= joy->axes[5];
	subscriber_right_steer_ 		= joy->buttons[11] ;		
	subscriber_left_steer_ 			= joy->buttons[12] ;
	subscriber_buttonA_ 			= joy->buttons[0];
	subscriber_buttonB_ 			= joy->buttons[1];
	
	if ( subscriber_buttonA_ ) {remote_control_ = true; opencv_ = false; feedback_xmega_ = RC_COMMAND;camera_steering_ = 's';}
    if ( subscriber_buttonB_ ) {remote_control_ = false; feedback_xmega_ = ERROR_FREE; }
}

void robot::pan_camera_servo(){
	camera_steering_ = camera_temp_;
	pan_counter_--;
	if (pan_counter_ == PAN_VALUE/2) camera_temp_ = 'm';
	else if(!pan_counter_) {pan_counter_ = PAN_VALUE; camera_temp_ = 'p';}
}

void serial::transmit_to_serial(char motor, char servo, char camera_steering, char error_xmega){
	mySerial.write(&motor, 1);
	mySerial.write(&servo, 1);
	mySerial.write(&camera_steering, 1);
	mySerial.write(&error_xmega, 1);
}
