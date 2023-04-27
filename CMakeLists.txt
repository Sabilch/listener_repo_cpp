cmake_minimum_required(VERSION 3.0.2)
project(sabilpub)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  genmsg
  roscpp
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   person_data.msg  
 )

generate_messages(
  DEPENDENCIES
  std_msgs 
  
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_program
  CATKIN_DEPENDS roscpp std_msgs
)

add_executable(sabilpub src/sabilpub.cpp)
add_dependencies(sabilpub ${sabilpub_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( sabilpub ${catkin_LIBRARIES} )


 install(TARGETS sabilpub
   RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
 
 
## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING
  PATTERN "*.launch"
)




include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)






