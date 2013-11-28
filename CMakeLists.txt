cmake_minimum_required(VERSION 2.8.3)
project(robotBrain)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  cv_bridge
  image_transport
 )

find_package(
 OpenCV REQUIRED  
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   #joystickMessage.msg
   #joy.msg
   #feedbackMessage.msg
   opencvMessage.msg
 )


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES communication
  #CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime #joy	
#  DEPENDS ${OpenCV_LIBS} ${catkin_EXPORTED_TARGETS}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include 
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
## Specify libraries to link a library or executable target against
 #add_executable(joyTalker src/joy.cpp)
 #target_link_libraries(joyTalker
 #  ${catkin_LIBRARIES}
 #)

 add_executable(atxmegaTalker src/atxmegaTalker.cpp)
target_link_libraries(atxmegaTalker
   ${catkin_LIBRARIES}
   /usr/local/lib/libserial.so
 )
 
 add_executable(calibrate src/calibrate.cpp)
 target_link_libraries(calibrate
  ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
 )

 add_executable(face_detect src/face_detect.cpp)
 target_link_libraries(face_detect
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
 )

add_executable(image_comm_sub_node src/image_comm_sub_node.cpp)
 target_link_libraries(image_comm_sub_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
 )

