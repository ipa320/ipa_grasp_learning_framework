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

#ifndef GRASPPOSE_SAMPLER_H
#define GRASPPOSE_SAMPLER_H


#include <ros/ros.h>
#include <ipa_grasppose_srvs/GetGraspPoses.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>


class GraspposeSampler
{
protected:

	bool initialized_;
	std::vector<double> conical_sampling_;
	std::vector<double> circular_sampling_;
	std::vector<double> grasp_depth_sampling_;

	// Samples further vectors (candidates) in a conical fashion around the surface normal (in camera frame here)
	// The surface normal points from the peak of the cone towards the center of the base circle of the cone
	void sampleCandidateVectorsConically(const Eigen::Vector3d &normal, std::vector<Eigen::Vector3d> &samples_out);

	// Samples further grasp frames around the alignment direction:
	// 1. Each sampled candidate vector (inverted) from above marks an alignment direction
	// 2. The grasp direction of the gripper will be aligned with each alignment direction (and with the 2nd alignment condition when specified) which results in the grasppose base frame
	// 3. The grasppose base frame will then be rotated around the grasp direction to sample further candidate frames
	void sampleCandidateFramesAroundGraspDirection(const Eigen::Quaterniond &grasp_base_frame, const Eigen::Vector3d &alignment_vector, std::vector<Eigen::Quaterniond> &grasp_frame_candidates);


public:

	GraspposeSampler(const ros::NodeHandle &nh);
	bool getGraspPoses(ipa_grasppose_srvs::GetGraspPoses::Request &request, ipa_grasppose_srvs::GetGraspPoses::Response &response);

};

#endif // GRASPPOSE_SAMPLER_H
