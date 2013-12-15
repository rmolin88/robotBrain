cmake_minimum_required(VERSION 2.8.3)
project(robot_brain)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  roscpp
  sensor_msgs
  image_transport
)

find_package(
 OpenCV REQUIRED  
)

#generate_messages(
#   DEPENDENCIES
#   sensor_msgs
#)

add_message_files(
   #FILES
   #joystickMessage.msg
   #joy.msg
   #feedbackMessage.msg
   #opencvMessage.msg
 )

catkin_package()


include_directories(inc
	${catkin_INCLUDE_DIRS}
)

 add_executable(robot_main_node src/robot_main_node.cpp)
 add_library(robot ~/ROS/catkin_ws/src/robot_brain/src/robot.cpp)
 target_link_libraries(robot_main_node
   ${catkin_LIBRARIES}
   /usr/local/lib/libserial.so
   robot
 )
 
