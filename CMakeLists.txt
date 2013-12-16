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

add_message_files(
   FILES
   opencv.msg
 )

generate_messages(
   DEPENDENCIES
   sensor_msgs
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
 
