/*
 * This file is part of ipa_graspdata_processor.
 * Copyright (C) 2021  Marc Riedlinger

 * ipa_graspdata_processor is free software: you can redistribute it and/or modify
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

// Ros node which is responsible for processing grasp-related data for a NN


#include <ipa_graspdata_processor/graspdata_processor.h>
#include <ros/ros.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "ipa_graspdata_processor");
	ros::NodeHandle node_handle("~");  // create a private node handle

	GraspdataProcessor processor(node_handle);
	ros::ServiceServer service = node_handle.advertiseService("process_graspdata_srv", &GraspdataProcessor::processGraspData, &processor);

	ros::spin();
	return 0;
}
