cmake_minimum_required(VERSION 2.8.3)
project(summit_xl_navigation)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	move_base_msgs
	actionlib
	geometry_msgs
	tf
)

catkin_package( 
	CATKIN_DEPENDS roscpp move_base_msgs actionlib geometry_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

install(
  DIRECTORY launch config maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(sendGoals1 src/sendGoals.cpp)

add_dependencies(sendGoals1 robot_navigation_gencpp)

#This will link executables to the appropriate libraries
target_link_libraries(sendGoals1 ${catkin_LIBRARIES})
