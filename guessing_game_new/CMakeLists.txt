cmake_minimum_required(VERSION 2.8.3)
project(guessing_game_new)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
	/usr/local/lib/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
	sensor_msgs
  cv_bridge
  image_transport
  message_generation
)

# find_package(aruco REQUIRED )
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

##############################################################################
# Messages, Services and Actions
##############################################################################
add_message_files(
   DIRECTORY msg
   FILES
   # BlobCenter.msg
)

## Generate services
add_service_files(
	DIRECTORY srv
  FILES
  text_to_speech.srv
  speech_to_text.srv
  image_request.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
	sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package()
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}

)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
## Declare a cpp executable
#add_executable(vision_node src/vision_node.cpp)
#target_link_libraries(vision_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${aruco_LIBS})

add_executable(clustering_node src/clustering.cpp)
target_link_libraries(clustering_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(visualize_node src/visualize.cpp)
#target_link_libraries(visualize_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

catkin_install_python(PROGRAMS scripts/speech.py  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/vision.py  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/roboy.py  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/guessing_game.py  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})