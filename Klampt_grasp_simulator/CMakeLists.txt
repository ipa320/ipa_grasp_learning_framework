cmake_minimum_required(VERSION 2.8.3)
project(Klampt_grasp_simulator)
add_compile_options(-std=c++11)

set(KLAMPT_ROOT "~/Programs/Klampt")
set (CMAKE_MODULE_PATH "${KLAMPT_ROOT}/CMakeModules")

set(catkin_RUN_PACKAGES			# all ROS packages from package.xml (libopencv-dev is system dependency --> sudo apt-get install)
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

find_package(Klampt REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem)


###################################
## catkin specific configuration ##
###################################

catkin_package(
INCLUDE_DIRS
	ros/include
	common/include
LIBRARIES
CATKIN_DEPENDS
	${catkin_RUN_PACKAGES}
DEPENDS
	KLAMPT
	Boost
)

###########
## Build ##
###########

add_definitions(${KLAMPT_DEFINITIONS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	ros/include
	common/include
	${catkin_INCLUDE_DIRS} 		# same name as find_package
	${KLAMPT_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

add_executable(Klampt_grasp_simulator	ros/src/Klampt_grasp_simulator_node.cpp
										ros/src/simulation_backend.cpp
										ros/src/gripper_interface_base.cpp
										ros/src/gripper_interface_cob4_old.cpp
										common/src/view_program_base.cpp
										common/src/grasp_view_program.cpp
										common/src/calculation_utilities.cpp
										common/src/file_utilities.cpp
)

target_link_libraries(Klampt_grasp_simulator
		${catkin_LIBRARIES} # automatically links all catkin_BUILD_PACKAGES
		${KLAMPT_LIBRARIES}
		${Boost_LIBRARIES}
)

add_dependencies(Klampt_grasp_simulator	${catkin_EXPORTED_TARGETS})
