/*
 * This file is part of ipa_graspdata_manager.
 * Copyright (C) 2021  Marc Riedlinger

 * ipa_graspdata_manager is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

// Ros node which is responsible for saving/loading grasp-related data


#include <ipa_graspdata_manager/graspdata_manager.h>
#include <ros/ros.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "ipa_graspdata_saver");
	ros::NodeHandle node_handle("~");  // create a private node handle
	GraspdataManager manager(node_handle);

	ros::AsyncSpinner spinner(4);  // Use 4 threads to cover service calls of multiple nodes at once
	ros::Rate loop_rate(1);  // 1 Hz -> save new data once a second
	spinner.start();
	while ( ros::ok() )
	{
		manager.saveCurrentGraspQueue();
		loop_rate.sleep();
	}
	spinner.stop();
	return 0;
}
