/*
 * This file is part of ipa_grasppose_sampler.
 * Copyright (C) 2021  Marc Riedlinger

 * ipa_grasppose_sampler is free software: you can redistribute it and/or modify
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

// Ros node which is responsible for generating grasp pose samples on arbitrary point cloud data


#include <ipa_grasppose_sampler/grasppose_sampler.h>
#include <ros/ros.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "ipa_grasppose_sampler");
	ros::NodeHandle node_handle("~");  // create a private node handle

	GraspposeSampler sampler(node_handle);
	ros::ServiceServer service = node_handle.advertiseService("sample_graspposes_srv", &GraspposeSampler::getGraspPoses, &sampler);

	ros::spin();
	return 0;
}
