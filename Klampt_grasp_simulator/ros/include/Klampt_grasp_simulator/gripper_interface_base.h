/*
 * This file is part of Klampt_grasp_simulator.
 * Copyright (C) 2021  Marc Riedlinger

 * Klampt_grasp_simulator is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef GRIPPER_INTERFACE_BASE_H
#define GRIPPER_INTERFACE_BASE_H


#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math3d/rotation.h>
#include <KrisLibrary/robotics/RobotKinematics3D.h>
#include <Klampt/Modeling/World.h>
#include <Klampt/Simulation/WorldSimulation.h>
#include <Klampt/Modeling/Robot.h>
#include <Klampt/Control/Controller.h>
#include <Klampt/Simulation/ControlledSimulator.h>
#include <Klampt/Interface/SimulationGUI.h>
#include <ros/ros.h>
#include <string>
#include <cmath>


class GripperInterfaceBase
{
protected:

	RobotWorld &world_;
	WorldSimulation &simulation_;
	int gripper_index_;
	shared_ptr<Robot> gripper_;
	shared_ptr<RobotController> controller_;
	ControlledRobotSimulator* controlled_simulator_;
	std::vector<size_t> gripper_finger_frame_indexes_;
	std::vector<size_t> revolute_joint_indexes_;
	size_t gripper_grasp_frame_index_;
	Math3D::Vector3 grasp_point_position_;
	Config gripper_opened_config_;
	Config gripper_closed_config_;
	double contact_force_threshold_;
	double global_gripper_movement_timeout_;
	double pre_grasp_offset_;
	double object_approaching_move_time_;
	Math3D::Vector3 grasp_direction_vector_;
	Math3D::Vector3 gripper_alignment_vector_;
	bool initialized_;


public:

	GripperInterfaceBase(ros::NodeHandle* nh, RobotWorld &world, WorldSimulation &simulation);
	virtual ~GripperInterfaceBase();
	virtual Config getCurrentConfig();
	virtual Config getGripperOpenedConfig();
	virtual Config getGripperClosedConfig();
	virtual bool isGripperOpen();  // deprecated
	virtual bool isGripperClosed();
	virtual bool setMoveToConfig(Config &target_config);
	virtual bool setMoveToCartesian(const RigidTransform &target_pose, Config &target_config,
									const double &move_time = 0, SimGUIBackend* gui_backend = nullptr);
	virtual bool computeConfigFromCartesian(const RigidTransform &target_pose, Config &target_config);
	virtual void setOpenGripper();  // deprecated
	virtual void addOpenGripperToConfig(Config &append_to);
	virtual void stopMovement();
	virtual Math3D::Vector3 getGraspDirectionVector();
	virtual Math3D::Vector3 getGripperAlignmentVector();
	virtual double getPreGraspOffset();
	virtual double getObjectApproachingMoveTime();
	virtual double getGripperMovementTimeout(int simulation_state);
	virtual int getGripperID();
	virtual bool hasContactWithRigidObject();
	virtual bool maxContactForceExceeded();
	virtual std::vector<size_t> getRevoluteJointIndexes();

	// Check whether a target configuration has been reached by using getPerJointAccuracyParameters.
	// ignore_fingers is needed because the finger motors might have a target configuration,
	// that is not directly reachable but needed to generate pressure (e.g. when lifting something)
	virtual bool hasReachedTarget(const Config &target_config, const bool &ignore_fingers = false);

	virtual void stepCloseGripper(Config &current_gripper_target) = 0;  // gradually close gripper "step by step". takes current_gripper_target and updates it with current finger position
	virtual void setGraspValidationMovement(Config &target_config) = 0;
	virtual bool isPoseTooCloseToTerrain(const Math3D::RigidTransform &pose) = 0;  // returns wether given pose is too close to a terrain, one of the checks used to trigger the evasion movement
	virtual void setTerrainEvasionMovement(const Math3D::RigidTransform target_pose, Config &target_config) = 0;  // modifiy current gripper position when it has the chance to get stuck with terrain geometry
	virtual Config getPerJointAccuracyParameters() = 0;  // get the allowed deviation for each joint in order for two configs to be considered equal
	virtual bool isInGraspableReach(const double &distance_to_grasppose_frame_along_grasp_direction) = 0;  // the distance value can be positive or negative depending on whether the considered point is in front or behind the related grasppose frame

	virtual bool isInitialized();
	virtual void getWorldToGraspFrame(Math3D::RigidTransform &world_to_grasp_frame);

	static GripperInterfaceBase* createGripperInterfaceByName(const std::string &interface_name, ros::NodeHandle* nh, RobotWorld &world, WorldSimulation &simulation);
};

#endif // GRIPPER_INTERFACE_BASE_H
