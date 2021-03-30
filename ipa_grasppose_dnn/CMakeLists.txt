cmake_minimum_required(VERSION 2.8.3)
project(ipa_grasppose_dnn)

set(catkin_RUN_PACKAGES
	cv_bridge
	geometry_msgs
	ipa_grasppose_srvs
	roslib
	rospy
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

catkin_python_setup()

catkin_package(
#INCLUDE_DIRS
	#ros/include
#CATKIN_DEPENDS
#	${catkin_RUN_PACKAGES}
#DEPENDS
#	OpenCV
)

include_directories(
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

install(
	PROGRAMS
		nodes/ipa_predict_graspquality
		scripts/ipa_train_grasp_dnn
		scripts/ipa_create_validation_dataset
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


