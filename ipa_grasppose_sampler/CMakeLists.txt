cmake_minimum_required(VERSION 2.8.3)
project(ipa_grasppose_sampler)
add_compile_options(-std=c++11)


set(catkin_RUN_PACKAGES			# all ROS packages from package.xml (libopencv-dev is system dependency --> sudo apt-get install)
	ipa_grasppose_srvs
	pcl_ros
	roscpp
	roslib
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

find_package(PCL 1.2 REQUIRED)

find_package(Eigen3 REQUIRED)


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
	EIGEN3
	PCL
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
	${PCL_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)

#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

add_executable(ipa_grasppose_sampler	ros/src/ipa_grasppose_sampler_node.cpp
										ros/src/grasppose_sampler.cpp
										common/src/calculation_utilities.cpp
)

target_link_libraries(ipa_grasppose_sampler
	${catkin_LIBRARIES} # automatically links all catkin_BUILD_PACKAGES
	${PCL_LIBRARIES}
)

add_dependencies(ipa_grasppose_sampler	${catkin_EXPORTED_TARGETS})
