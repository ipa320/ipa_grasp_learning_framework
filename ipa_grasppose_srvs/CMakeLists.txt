cmake_minimum_required(VERSION 2.8.3)
project(ipa_grasppose_srvs)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
	geometry_msgs
	message_generation
	sensor_msgs
)


#######################################
## Declare ROS messages and services ##
#######################################

add_service_files(FILES 
	GetGraspPoses.srv
	SaveGraspData.srv
	ProcessGraspData.srv
	PredictGraspQualities.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
	geometry_msgs
	sensor_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS
	geometry_msgs
	message_runtime
	sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	${catkin_INCLUDE_DIRS} 		# same name as find_package
)
