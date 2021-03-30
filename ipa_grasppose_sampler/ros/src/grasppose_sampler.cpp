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

#include <ipa_grasppose_sampler/grasppose_sampler.h>
#include <ipa_grasppose_sampler/calculation_utilities.h>
#include <geometry_msgs/Pose.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/pca.h>
#include <algorithm>
#include <cmath>


GraspposeSampler::GraspposeSampler(const ros::NodeHandle &nh) :
					initialized_(false)
{
	nh.getParam("conical_sampling", conical_sampling_);
	if ( conical_sampling_.size() != 6 )
	{
		ROS_ERROR("GraspposeSampler::GraspposeSampler - conical_sampling has wrong size %d, expected 6.", (int)conical_sampling_.size());
		return;
	}

	nh.getParam("circular_sampling", circular_sampling_);
	if ( circular_sampling_.size() != 3 )
	{
		ROS_ERROR("GraspposeSampler::GraspposeSampler - circular_sampling has wrong size %d, expected 3.", (int)circular_sampling_.size());
		return;
	}

	nh.getParam("grasp_depth_sampling", grasp_depth_sampling_);
	if ( grasp_depth_sampling_.size() != 3 )
	{
		ROS_ERROR("GraspposeSampler::GraspposeSampler - grasp_depth_sampling has wrong size %d, expected 3.", (int)grasp_depth_sampling_.size());
		return;
	}

	double &cone_theta_start = conical_sampling_[0];
	double &cone_theta_step = conical_sampling_[1];
	double &cone_theta_end = conical_sampling_[2];
	double &cone_phi_start = conical_sampling_[3];
	double &cone_phi_step = conical_sampling_[4];
	double &cone_phi_end = conical_sampling_[5];
	double &gamma_start = circular_sampling_[0];
	double &gamma_step = circular_sampling_[1];
	double &gamma_end = circular_sampling_[2];
	double &depth_start = grasp_depth_sampling_[0];
	double &depth_step = grasp_depth_sampling_[1];
	double &depth_end = grasp_depth_sampling_[2];

	// Fix range parameters if needed
	if ( !CalcUtils::forceRangeMinMax(cone_theta_start, 0, M_PI_2) )
		ROS_WARN("GraspposeSampler::GraspposeSampler - conical_sampling's theta start exceeds [0,pi/2], set to comply with boundary.");

	if ( !CalcUtils::forceRangeMinMax(cone_theta_end, 0, M_PI_2) )
		ROS_WARN("GraspposeSampler::GraspposeSampler - conical_sampling's theta end exceeds [0,pi/2], set to comply with boundary.");

	if ( !CalcUtils::forceRangeAbs(cone_phi_start, M_PI) )
		ROS_WARN("GraspposeSampler::GraspposeSampler - conical_sampling's phi start exceeds [-pi,pi], set to comply with boundary.");

	if ( !CalcUtils::forceRangeAbs(cone_phi_end, M_PI) )
		ROS_WARN("GraspposeSampler::GraspposeSampler - conical_sampling's phi end exceeds [-pi,pi], set to comply with boundary.");

	if ( !CalcUtils::forceRangeMinMax(gamma_start, 0, M_PI*2) )
		ROS_WARN("GraspposeSampler::GraspposeSampler - circular_sampling's gamma start exceeds [0,2*pi], set to comply with boundary.");

	if ( !CalcUtils::forceRangeMinMax(gamma_end, 0, M_PI*2) )
		ROS_WARN("GraspposeSampler::GraspposeSampler - circular_sampling's gamma end exceeds [0,2*pi], set to comply with boundary.");

	if ( !CalcUtils::checkAndFixStep(cone_theta_start, cone_theta_end, cone_theta_step) )
		ROS_WARN("GraspposeSampler::GraspposeSampler - conical_sampling's theta range is badly configured, negating step value.");

	if ( !CalcUtils::checkAndFixStep(cone_phi_start, cone_phi_end, cone_phi_step) )
		ROS_WARN("GraspposeSampler::GraspposeSampler - conical_sampling's phi range is badly configured, negating step value.");

	if ( !CalcUtils::checkAndFixStep(gamma_start, gamma_end, gamma_step) )
		ROS_WARN("GraspposeSampler::GraspposeSampler - circular_sampling's gamma range is badly configured, negating step value.");

	if ( !CalcUtils::checkAndFixStep(depth_start, depth_end, depth_step) )
		ROS_WARN("GraspposeSampler::GraspposeSampler - grasp_depth_sampling's depth range is badly configured, negating step value.");

	// Theta = 0 matches with the surface normal which will already be considered, so start with theta > 0
	if ( cone_theta_start < 0.0001 && cone_theta_step > 0.0 )
		cone_theta_start += cone_theta_step;

	// Theta = 0 matches with the surface normal which will already be considered, so end with theta > 0
	if ( cone_theta_end < 0.0001 && cone_theta_step < 0.0 )
		cone_theta_end -= cone_theta_step;

	// Gamma = 0 matches with the surface normal which will already be considered, so start with Gamma > 0
	if ( gamma_start < 0.0001 && gamma_step > 0.0 )
		gamma_start += gamma_step;

	// Gamma = 0 matches with the surface normal which will already be considered, so end with Gamma > 0
	if ( gamma_end < 0.0001 && gamma_step < 0.0 )
		gamma_end -= gamma_step;

	initialized_ = true;
}

// Computes grasp poses in the target frame. This is done by two seperate rotations for each chosen candidate (= normal or conical variations).
// The first rotation aligns the grasp_direction_vector with the chosen candidate.
// The second rotation aligns the gripper_alignment_vector with the PCA axis of the object that yields the most geometric inliers (this rotation is optional).
// This rotation is done by rotating around the chosen candidate by a specific angle to match the direction of both vectors.
// To turn off the second rotation, set gripper_alignment_vector_intermediate = (0,0,0).
bool GraspposeSampler::getGraspPoses(ipa_grasppose_srvs::GetGraspPoses::Request &request, ipa_grasppose_srvs::GetGraspPoses::Response &response)
{
	if ( !initialized_ )
	{
		ROS_ERROR("GraspposeSampler::getGraspPoses - Grasp sampler has not sucessfully been initialized.");
		return false;
	}

	// 1. Convert to pcl pointcloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(request.point_cloud, *pcl_pointcloud);

	if ( pcl_pointcloud->points.empty() )
	{
		ROS_WARN("GraspposeSampler::getGraspPoses - Given point cloud is empty.");
		return false;
	}

	// 2. Calculate surface normals (copied from: http://pointclouds.org/documentation/tutorials/normal_estimation.php)
	// Do this on the dense point cloud instead on the downsampled one to preserve the best quality of the normals
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation;
	normal_estimation.setInputCloud(pcl_pointcloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	normal_estimation.setSearchMethod(tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of given radius
	normal_estimation.setRadiusSearch(request.normal_search_radius);

	// Compute the features
	normal_estimation.compute(*cloud_normals);

	// Save potential bad entries to ignore them later on
	std::vector<size_t> invalid_normals_indexes;
	for ( size_t i=0; i<cloud_normals->points.size(); ++i )
	{
		const Eigen::Vector3f normal(cloud_normals->points[i].getNormalVector3fMap());
		if ( std::isnan(normal.x()) || std::isnan(normal.y()) || std::isnan(normal.z()) )
			invalid_normals_indexes.push_back(i);
	}

	// 3. Downsample the point cloud using a voxel grid. This is done to reduce the count of the normals and to perform a faster PCA.
	bool use_downsampling = true;
	const float &leaf_size = request.downsampling_voxel_leaf_size;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
	if ( leaf_size > 0.001f )
	{
		pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
		voxel_grid.setInputCloud(pcl_pointcloud);
		voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
		voxel_grid.filter(*pcl_pointcloud_downsampled);
	}
	else  // no downsampling
	{
		use_downsampling = false;
		pcl_pointcloud_downsampled = pcl_pointcloud;
	}

	// 4. Match each of the downsampled point cloud points with the closest normal and build a reduced normals set
	std::vector<size_t> normal_index_set;  // set of normals that have been selected from original point cloud
	if ( use_downsampling )
	{
		for ( pcl::PointXYZ pcl_point : pcl_pointcloud_downsampled->points ) //size_t i=0; i<pcl_pointcloud_downsampled->points.size(); ++i )
		{
			const Eigen::Vector3f surface_point_sampled(pcl_point.getArray3fMap());
			double minimal_distance_sq = -1;
			size_t minimal_index = 0;

			for ( size_t j=0; j<pcl_pointcloud->points.size(); ++j )  // pcl_pointcloud has same size as cloud_normals
			{
				if ( std::find(invalid_normals_indexes.begin(), invalid_normals_indexes.end(), j) != invalid_normals_indexes.end() )  // exclude invalid entries
					continue;

				const Eigen::Vector3f surface_point(pcl_pointcloud->points[j].getArray3fMap());
				const double distance_sq = (surface_point - surface_point_sampled).squaredNorm();

				if ( distance_sq < leaf_size*leaf_size && (minimal_distance_sq < 0 || distance_sq < minimal_distance_sq) )  // retrieve point cloud point within the current voxel with minimal distance to sampled point
				{
					minimal_distance_sq = distance_sq;
					minimal_index = j;
				}
			}

			if ( minimal_distance_sq > 0.0 )  // found an entry
				normal_index_set.push_back(minimal_index);
		}
	}
	else
	{
		normal_index_set.reserve(pcl_pointcloud->points.size());
		for ( size_t i=0; i<pcl_pointcloud->points.size(); ++i )
		{
			if ( std::find(invalid_normals_indexes.begin(), invalid_normals_indexes.end(), i) != invalid_normals_indexes.end() )  // exclude invalid entries
				continue;

			normal_index_set.push_back(i);
		}
	}

	// 5. Perform a PCA upon the downsampled point cloud to retrieve main directions for aligning the gripper accordingly
	const Eigen::Vector3d gripper_alignment_vector_grasp_frame(request.gripper_alignment_vector_grasp_frame.x, request.gripper_alignment_vector_grasp_frame.y,
																request.gripper_alignment_vector_grasp_frame.z);
	bool do_second_alignment = false;
	Eigen::Matrix3d eigenvectors_cam;
	if ( !gripper_alignment_vector_grasp_frame.isZero() )  // perform the 2nd alignment
	{
		do_second_alignment = true;
		pcl::PointCloud<pcl::PointXYZ>::Ptr pca_projection(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PCA<pcl::PointXYZ> pca;
		pca.setInputCloud(pcl_pointcloud_downsampled);
		pca.project(*pcl_pointcloud_downsampled, *pca_projection);
		eigenvectors_cam = pca.getEigenVectors().cast<double>();  // eigenvectors in camera frame
	}

	// 6. Create grasppose base frames from selected normals and their candidate vectors built upon target frame
	// Sample further candidate frames by rotating around each alignment vector (gripper rolling)
	const Eigen::Matrix4d target_to_camera_frame_trafo = CalcUtils::geometryPoseToEigenMatrix(request.target_to_camera_transform);
	const Eigen::Matrix3d target_to_camera_frame_rot = target_to_camera_frame_trafo.block<3,3>(0,0);
	const Eigen::Vector3d grasp_direction_vector_grasp_frame(request.grasp_direction_vector_grasp_frame.x, request.grasp_direction_vector_grasp_frame.y,
														request.grasp_direction_vector_grasp_frame.z);

	// Depth sampling which is done as last step
	const double &depth_start = grasp_depth_sampling_[0];
	const double &depth_step = grasp_depth_sampling_[1];
	const double &depth_end = grasp_depth_sampling_[2];

	for ( size_t i=0; i<normal_index_set.size(); ++i )
	{
		const Eigen::Vector3d normal_cam(cloud_normals->points[normal_index_set[i]].getNormalVector3fMap().cast<double>());  // normal in camera frame coordinates
		const Eigen::Vector3d surface_point_cam(pcl_pointcloud->points[normal_index_set[i]].getVector3fMap().cast<double>());  // surface point correspondig to normal in camera frame coordinates
		Eigen::Vector3d best_pca_eigenvector_target;
		if ( do_second_alignment )
			best_pca_eigenvector_target = target_to_camera_frame_rot * CalcUtils::determineBestEigenvector(pcl_pointcloud_downsampled, eigenvectors_cam, surface_point_cam, leaf_size);  // eigenvector with most inliers in target frame

		std::vector<Eigen::Vector3d> candidate_vector_samples_cam;  // further samples in camera frame
		sampleCandidateVectorsConically(normal_cam, candidate_vector_samples_cam);

		// Transform grasp origin to target frame
		const Eigen::Vector3d grasp_origin_target = target_to_camera_frame_rot*surface_point_cam + target_to_camera_frame_trafo.block<3,1>(0,3);  // Affine transformation: R*x + t

		for ( Eigen::Vector3d candidate_cam : candidate_vector_samples_cam )  // create a grasp frame for each candidate
		{
			// Invert candidate, so that it is pointing towards the surface
			const Eigen::Vector3d candidate_inverted_cam = candidate_cam * -1;

			// Transform inverted normal to target frame
			const Eigen::Vector3d candidate_inverted_target = target_to_camera_frame_rot * candidate_inverted_cam;

			// Compute rotation around rotation axis using two quaternion rotations
			Eigen::Quaterniond target_to_grasppose_rotation;
			target_to_grasppose_rotation.setFromTwoVectors(grasp_direction_vector_grasp_frame, candidate_inverted_target);  // 1. Rotation: Align given vector with inverted candidate

			// Try to achieve another alignment so that the gripper aligns with the most suitable eigenvector of the PCA
			// The second rotation will be done in an angle-axis rotation: Rotate around the candidate axis until the gripper_alignment_vector_grasp_frame lies in the candidate_axis-pca_axis-plane.
			if ( do_second_alignment )
			{
				const Eigen::Matrix3d intermediate_to_target = target_to_grasppose_rotation.matrix().inverse();  // the first alignment creates an intermediate or temporary frame because there will be a second alignment on top which results then in the final target_to_grasppose_rotation
				// Project the gripper_alignment_vector_grasp_frame along the grasp_direction_vector_grasp_frame until they are perpendicular and return that vector.
				const Eigen::Vector3d gripper_alignment_vector_orthogonal_intermediate = CalcUtils::projectVectorUntilPerpendicular(grasp_direction_vector_grasp_frame, gripper_alignment_vector_grasp_frame);  // grasp_direction_vector_grasp_frame == candidate_inverted_intermediate after first alignment
				const Eigen::Vector3d max_eigen_vector_intermediate = intermediate_to_target * best_pca_eigenvector_target;
				const Eigen::Vector3d candidate_pca_plane_normal_intermediate = grasp_direction_vector_grasp_frame.cross(max_eigen_vector_intermediate);  // get a vector that is perpendicular to grasp_direction_vector_grasp_frame
				// The max_eigen_vector_intermediate is not necessarily orthogonal to grasp_direction_vector_grasp_frame, this is ensured by the following cross product.
				const Eigen::Vector3d projected_vector_intermediate = grasp_direction_vector_grasp_frame.cross(candidate_pca_plane_normal_intermediate);  // resulting vector is orthogonal to candidate-pca plane normal and the candidate vector itself
				// Now that we have two orthogonal vectors to the rotation axis (grasp_direction_vector_grasp_frame), get the angle between them which is the needed rotation angle
				target_to_grasppose_rotation *= Eigen::Quaterniond(CalcUtils::getAngleAxisFromVectors(gripper_alignment_vector_orthogonal_intermediate, projected_vector_intermediate));  // align gripper along best PCA eigenvector
			}

			std::vector<Eigen::Quaterniond> grasppose_candidates_target;  // further graspposes in target coordinates
			sampleCandidateFramesAroundGraspDirection(target_to_grasppose_rotation, grasp_direction_vector_grasp_frame, grasppose_candidates_target);

			// Construct pose messages and push them to the response vector
			for ( Eigen::Quaterniond rotation : grasppose_candidates_target )
			{
				// Third sampling method
				for ( double depth=depth_start; (depth_step >= 0.0) ? depth<=depth_end : depth>=depth_end; depth+=depth_step )
				{
					const Eigen::Vector3d grasp_origin_shifted_target = grasp_origin_target + candidate_inverted_target*depth;
					geometry_msgs::Pose grasp_transform;
					grasp_transform.position.x = grasp_origin_shifted_target.x();
					grasp_transform.position.y = grasp_origin_shifted_target.y();
					grasp_transform.position.z = grasp_origin_shifted_target.z();
					grasp_transform.orientation.w = rotation.w();
					grasp_transform.orientation.x = rotation.x();
					grasp_transform.orientation.y = rotation.y();
					grasp_transform.orientation.z = rotation.z();
					response.grasp_poses_target_frame.push_back(grasp_transform);
				}
			}
		}
	}

	ROS_WARN("[Service] GraspposeSampler::getGraspPoses - %d grasp poses generated.", (int)response.grasp_poses_target_frame.size());
	return true;
}

// First sampling method
void GraspposeSampler::sampleCandidateVectorsConically(const Eigen::Vector3d &normal, std::vector<Eigen::Vector3d> &samples_out)
{
	samples_out.push_back(normal);  // first sample is the normal itself

	Eigen::Quaterniond rotation_quaternion;  // rotation to a new frame where we can generate further candidates using spherical coordinates with a radius = 1
	rotation_quaternion.setFromTwoVectors(Eigen::Vector3d(0, 0, 1), normal);  // rotation from z-axis to normal
	Eigen::Matrix3d original_to_cone_frame = rotation_quaternion.matrix();  // from original frame of the input normal to local cone frame

	// Get samples by discretizing a cone around the normal using spherical coordinates
	// Theta is half the opening angle of the cone
	const double &cone_theta_start = conical_sampling_[0];
	const double &cone_theta_step = conical_sampling_[1];
	const double &cone_theta_end = conical_sampling_[2];
	const double &cone_phi_start = conical_sampling_[3];
	const double &cone_phi_step = conical_sampling_[4];
	const double &cone_phi_end = conical_sampling_[5];

	for ( double theta=cone_theta_start; (cone_theta_step >= 0.0) ? theta-cone_theta_end<0.0001 : theta-cone_theta_end>-0.0001; theta+=cone_theta_step )
	{
		for ( double phi=cone_phi_start; (cone_phi_step >= 0.0) ? phi-cone_phi_end<0.0001 : phi-cone_phi_end>-0.0001; phi+=cone_phi_step )
		{
			Eigen::Vector3d candidate_cone_frame;
			candidate_cone_frame.x() = /*radius * */sin(theta) * cos(phi);  // radius = 1
			candidate_cone_frame.y() = /*radius * */sin(theta) * sin(phi);
			candidate_cone_frame.z() = /*radius * */cos(theta);
			samples_out.push_back(original_to_cone_frame * candidate_cone_frame);  // transform back to original frame and store candidate
		}
	}
}

// Second sampling method
void GraspposeSampler::sampleCandidateFramesAroundGraspDirection(const Eigen::Quaterniond &grasp_base_frame, const Eigen::Vector3d &alignment_vector, std::vector<Eigen::Quaterniond> &grasp_frame_candidates)
{
	grasp_frame_candidates.push_back(grasp_base_frame);

	const double &gamma_start = circular_sampling_[0];
	const double &gamma_step = circular_sampling_[1];
	const double &gamma_end = circular_sampling_[2];

	for ( double gamma=gamma_start; (gamma_step >= 0.0) ? gamma-gamma_end<0.0001 : gamma-gamma_end>-0.0001; gamma+=gamma_step )
	{
		Eigen::Quaterniond new_rotation = grasp_base_frame * Eigen::Quaterniond(Eigen::AngleAxisd(gamma, alignment_vector));  // rotate around alignment_vector
		grasp_frame_candidates.push_back(new_rotation);
	}
}
