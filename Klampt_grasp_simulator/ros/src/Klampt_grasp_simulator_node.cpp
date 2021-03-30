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

// Main file that initializes the ros node, creates a private node handle and sets up a simulator


#include <Klampt_grasp_simulator/simulation_backend.h>
#include <Klampt_grasp_simulator/grasp_view_program.h>
#include <ros/ros.h>


int main(int argc, char** argv)
{
	// initialize ros node
	ros::init(argc, argv, "Klampt_grasp_simulator");
	ros::NodeHandle node_handle("~");  // create a private node handle

	bool create_gui = true;
	node_handle.param("create_gui", create_gui, true);

	RobotWorld world;
	SimulationBackend sim_backend(&node_handle, &world);
	GraspViewProgram program(sim_backend);
	if ( !program.loadAndInitSim() )
	{
		ROS_ERROR("Node Klampt_grasp_simulator, in main - Failed to setup simulation!");
		return -1;
	}

	if ( create_gui )
		return program.Run();
	else
		return program.runWithoutGUI();
}
