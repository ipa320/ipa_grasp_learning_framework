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

#include <Klampt_grasp_simulator/gripper_interface_cob4_old.h>
#include <Klampt_grasp_simulator/calculation_utilities.h>

#define GRIPPER_EVASION_Z_THREASHOLD 0.15  // threshold [in m] that contributes to triggering evasion movement when undercut
#define GRIPPER_GRASP_DEPTH_POSTIVE_END 0.06	// length of the interval [in m] in positive direction of the grasp direction vector (which is based on the grasp frame) that defines valid grasp points
#define GRIPPER_GRASP_DEPTH_NEGATIVE_END 0.03	// length of the interval [in m] in negative direction of the grasp direction vector (which is based on the grasp frame) that defines valid grasp points


GripperInterfaceCob4Old::GripperInterfaceCob4Old(ros::NodeHandle* nh, RobotWorld &world, WorldSimulation &simulation) :
					GripperInterfaceBase(nh, world, simulation)
{
}

GripperInterfaceCob4Old::~GripperInterfaceCob4Old()
{
}

void GripperInterfaceCob4Old::stepCloseGripper(Config &current_gripper_target)
{
	// Take current config as fallback when current_gripper_target is empty
	if ( current_gripper_target.empty() )
		controlled_simulator_->GetSimulatedConfig(current_gripper_target);

	double &link1 = current_gripper_target[gripper_finger_frame_indexes_[0]];
	double &link2 = current_gripper_target[gripper_finger_frame_indexes_[1]];

	const double link1_force = simulation_.ContactForce(world_.RobotLinkID(gripper_index_, gripper_finger_frame_indexes_[0])).norm();
	const double link2_force = simulation_.ContactForce(world_.RobotLinkID(gripper_index_, gripper_finger_frame_indexes_[1])).norm();
	const double max_force = std::max(link1_force, link2_force);

	if ( max_force >= contact_force_threshold_ )  // don't overdo
		return;

	const double step = (max_force < 0.01) ? 0.06 : 0.012;  // take a step size depending on whether we are in contact or not

	// Advance link 1 and keep relationsship between link1 and link2 in a way that link2 is always parallel to the gripper palm
	link1 = std::min(gripper_closed_config_[0], link1+step);  // do not undergo closed config
	link2 = std::max(gripper_closed_config_[1], 0.2-link1);

	CalcUtils::normalizeConfiguration(current_gripper_target, revolute_joint_indexes_);
	controller_->SendCommand("set_q", LexicalCast(current_gripper_target));
}

void GripperInterfaceCob4Old::setGraspValidationMovement(Config &target_config)
{
	if ( target_config.empty() )
		target_config = getCurrentConfig();

	target_config[2] += 0.3; // move 0.3 m up in z direction

	CalcUtils::normalizeConfiguration(target_config, revolute_joint_indexes_);
	setMoveToConfig(target_config);
}

bool GripperInterfaceCob4Old::isPoseTooCloseToTerrain(const Math3D::RigidTransform &pose)
{
	const Math3D::Vector3 &check_position = pose.t;
	const Config accuracy = getPerJointAccuracyParameters();
	return ((GRIPPER_EVASION_Z_THREASHOLD - check_position.z) > accuracy[2]);  // accuracy for motor that drives in z-direction
}

void GripperInterfaceCob4Old::setTerrainEvasionMovement(const Math3D::RigidTransform target_pose, Config &target_config)
{
	if ( target_config.empty() )
		target_config = getCurrentConfig();

	const double difference = GRIPPER_EVASION_Z_THREASHOLD - target_pose.t.z;  // move target_pose.z to GRIPPER_EVASION_Z_THREASHOLD which should be enough to avoid the gripper getting stuck at the ground terrain

	if ( difference > 0.0 )
		target_config[2] += difference;

	CalcUtils::normalizeConfiguration(target_config, revolute_joint_indexes_);
	setMoveToConfig(target_config);
}

Config GripperInterfaceCob4Old::getPerJointAccuracyParameters()
{
	std::vector<double> accuracy_requirements = {0.005, 0.005, 0.005, 0.015, 0.015, 0.015, 0.0001, 0.0001, 0.0001, 0.0001, 0.015, 0.015};
	return Config(accuracy_requirements);
}


bool GripperInterfaceCob4Old::isInGraspableReach(const double &distance_to_grasppose_frame_along_grasp_direction)
{
	// Check if the given distance value (sign is relative to the origin of the related grasppose frame) is within the specified interval (-GRIPPER_GRASP_DEPTH_NEGATIVE_END, GRIPPER_GRASP_DEPTH_POSTIVE_END)
	if ( distance_to_grasppose_frame_along_grasp_direction >= 0.0 )  // point is in front of grasppose frame -> in positive direction of the grasp direction vector
		return (distance_to_grasppose_frame_along_grasp_direction < GRIPPER_GRASP_DEPTH_POSTIVE_END);
	else
		return (distance_to_grasppose_frame_along_grasp_direction > -GRIPPER_GRASP_DEPTH_NEGATIVE_END);
}
