################################################################################
#
# CMakelist.txt to compile ROS nodes generated automatically (contact A. Berne if errors)
#
# Created by  : Alexandre Berne <alexandre.berne@cea.fr>
# Creation date: 05/07/2023
# version ='1.0'
#
# (c) CEA List - DRT/LIST/DSCIN/LCYL
#
################################################################################


cmake_minimum_required(VERSION 3.0.2)
project(cons)

SET(CMAKE_CXX_FLAGS  "-Wall -Wextra")

## Find a maximum of packages to have a generic CMakeLists
find_package(catkin COMPONENTS
  roscpp
  std_msgs
  image_transport
  cv_bridge
  camera_info_manager
  sensor_msgs
  message_generation
  topic_tools
)
find_package(Boost)
find_package(OpenCV)

## Adding custom messages and services
#add_message_files(
#  FILES
#)

## Add custom services if needed
#add_service_files(
#  FILES
#)

## Add custom messages if needed
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs image_transport cv_bridge camera_info_manager sensor_msgs message_generation topic_tools
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Code sources

add_executable(Cons polygraph_infra/Cons_infra.cpp user_code/Cons_user.cpp )
target_link_libraries(Cons ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(start_cons_ polygraph_infra/start_cons_.cpp )
target_link_libraries(start_cons_ ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})


# Install rules
install(TARGETS
  Cons
  start_cons_
  RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
 
 
## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING
  PATTERN "*.launch"
)