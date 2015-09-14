cmake_minimum_required(VERSION 2.8.3)
project(spin_the_bottle)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  actionlib
)

#It was necessary to add the additional cpp files
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES projecting_orienting
  CATKIN_DEPENDS cv_bridge roscpp image_transport sensor_msgs std_msgs
)

find_package(OpenCV)

include_directories(include ${OpenCV_INCLUDE_DIRS}}
)

#Example -> add_library(motion_orientation src/motionDetection.cpp src/LineProjectionE.cpp)

add_library(motion_orientation src/FaceDetection.cpp src/LineProjectionE.cpp src/WorldCoordinates.cpp src/PreGame.cpp src/ActionNaoNeck.cpp src/DetectFace.cpp src/ActionClientSay.cpp)
target_link_libraries(motion_orientation ${OpenCV_LIBRARIES})

# Added also the libraries here.
add_executable(subscriber src/my_subscriber.cpp)

#add_executable(subscriber src/my_subscriber.cpp src/LineProjectionE.cpp src/WorldCoordinates.cpp)

target_link_libraries(subscriber ${catkin_LIBRARIES} motion_orientation ${OpenCV_LIBRARIES}) #motion_orientation
