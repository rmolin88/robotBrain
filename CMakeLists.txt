cmake_minimum_required(VERSION 2.8.3)
project(robot_brain)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES opencv_calibrate
#  CATKIN_DEPENDS cv_bridge image_transport message_generation roscpp sensor_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

 add_executable(robot_main_node src/robot_main_node.cpp)
 target_link_libraries(robot_main_node
   ${catkin_LIBRARIES}
   /usr/local/lib/libserial.so
 )
