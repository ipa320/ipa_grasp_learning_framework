cmake_minimum_required(VERSION 2.8.3)
project(ipa_graspdata_manager)
add_compile_options(-std=c++11)


set(catkin_RUN_PACKAGES			# all ROS packages from package.xml (libopencv-dev is system dependency --> sudo apt-get install)
	cv_bridge
	geometry_msgs
	ipa_grasppose_srvs
	roscpp
	roslib
	sensor_msgs
)

set(catkin_BUILD_PACKAGES 
	${catkin_RUN_PACKAGES}
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
	${catkin_BUILD_PACKAGES}		# this makes ${catkin_LIBRARIES} include all libraries of ${catkin_BUILD_PACKAGES}
)

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem thread)


###################################
## catkin specific configuration ##
###################################

catkin_package(
INCLUDE_DIRS
	ros/include
	common/include
CATKIN_DEPENDS
	${catkin_RUN_PACKAGES}
DEPENDS
	OpenCV
	Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	ros/include
	common/include
	${catkin_INCLUDE_DIRS} 		# same name as find_package
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

add_executable(ipa_graspdata_manager	ros/src/ipa_graspdata_manager_node.cpp
										ros/src/graspdata_manager.cpp
										common/src/file_utilities.cpp
)

target_link_libraries(ipa_graspdata_manager
	${catkin_LIBRARIES} # automatically links all catkin_BUILD_PACKAGES
	${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}
)

add_dependencies(ipa_graspdata_manager	${catkin_EXPORTED_TARGETS})
