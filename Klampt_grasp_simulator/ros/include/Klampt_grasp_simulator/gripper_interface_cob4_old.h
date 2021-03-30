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

#ifndef GRIPPER_INTERFACE_H
#define GRIPPER_INTERFACE_H


#include <Klampt_grasp_simulator/gripper_interface_base.h>


class GripperInterfaceCob4Old : public GripperInterfaceBase
{
public:

	GripperInterfaceCob4Old(ros::NodeHandle* nh, RobotWorld &world, WorldSimulation &simulation);
	virtual ~GripperInterfaceCob4Old();

	virtual void stepCloseGripper(Config &current_gripper_target);
	virtual void setGraspValidationMovement(Config &target_config);
	virtual bool isPoseTooCloseToTerrain(const Math3D::RigidTransform &pose);
	virtual void setTerrainEvasionMovement(const Math3D::RigidTransform target_pose, Config &target_config);
	virtual Config getPerJointAccuracyParameters();
	virtual bool isInGraspableReach(const double &distance_to_grasppose_frame_along_grasp_direction);

};

#endif // GRIPPER_INTERFACE_H
