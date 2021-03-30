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

#include <ipa_grasppose_sampler/calculation_utilities.h>
#include <cmath>


namespace CalcUtils
{

bool forceRangeAbs(double &parameter, double const &border_abs)
{
	if ( std::abs(parameter) > border_abs )
	{
		parameter = (std::signbit(parameter) == true ? -border_abs : border_abs);
		return false;
	}

	return true;
}

bool forceRangeMinMax(double &parameter, double const &min, double const &max)
{
	if ( parameter < min )
	{
		parameter = min;
		return false;
	}
	else if ( parameter > max )
	{
		parameter = max;
		return false;
	}

	return true;
}

// Check and fix range parameters for for loops so that start and end move towards one another.
bool checkAndFixStep(double const &start, double const &end, double &step)
{
	const double difference = end - start;  // difference gives information about what the sign of the step value should be
	if ( std::abs(difference) < 0.000001 || std::abs(step) < 0.000001 )
		step = (std::signbit(difference) == true ? -1.0 : 1.0);  // set to 1.0 or -1.0 depending on what is needed to reach the end value

	if ( std::signbit(difference) != std::signbit(step) )
	{
		step *= -1;
		return false;
	}

	return true;
}

Eigen::Matrix4d geometryPoseToEigenMatrix(const geometry_msgs::Pose &pose)
{
	Eigen::Quaterniond eigen_quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
	Eigen::Matrix3d rotation_matrix = eigen_quaternion.matrix();
	Eigen::Matrix4d result;
	result.setIdentity();
	result.block<3,3>(0,0) = rotation_matrix;
	result.block<3,1>(0,3) = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
	return result;
}

Eigen::AngleAxisd getAngleAxisFromVectors(const Eigen::Vector3d &vector_from, const Eigen::Vector3d &vector_to)
{
	Eigen::Vector3d vect_1 = vector_from.normalized();
	Eigen::Vector3d vect_2 = vector_to.normalized();
	Eigen::Vector3d rotation_axis;
	rotation_axis = vect_1.cross(vect_2);
	rotation_axis.normalize();
	const double angle = std::acos(vect_1.dot(vect_2));
	return Eigen::AngleAxisd(angle, rotation_axis);
}

// Project project along project_along until both are perpendicular to each other
Eigen::Vector3d projectVectorUntilPerpendicular(const Eigen::Vector3d &project_along, const Eigen::Vector3d &project)
{
	if ( std::abs(project_along.dot(project)) < 0.0001 )  // already perpendicular
		return project;

	double scaling = -project_along.dot(project);
	return (project + scaling*project_along);
}

Eigen::Vector3d determineBestEigenvector(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, const Eigen::Matrix3d &eigenvectors, const Eigen::Vector3d &surface_point, const double &inlier_distance)
{
	Eigen::Vector3d result(0.0, 0.0, 0.0);
	size_t max_inliers = 0;

	// Build vector for faster access later on
	std::vector<Eigen::Vector3d> pcl_points;
	pcl_points.reserve(point_cloud->points.size());
	for ( pcl::PointXYZ pcl_point : point_cloud->points )
		pcl_points.push_back(pcl_point.getVector3fMap().cast<double>());

	// Calculate the distance from each pcl point to the line that establishes at the given surface_point along the eigenvectors
	// Calculate that way the amount of inliers for each eigenvector and take the one with the most
	for ( unsigned short i=0; i<3; ++i )
	{
		Eigen::Vector3d current_eigenvector = eigenvectors.col(i);
		size_t inlier_count = 0;

		for ( Eigen::Vector3d pcl_point : pcl_points )
		{
			const double scaling = (pcl_point-surface_point).dot(current_eigenvector);  // Scaling along eigenvector line so that distance vector to pcl_point is orthogonal
			const Eigen::Vector3d distance = pcl_point - (surface_point + scaling*current_eigenvector);

			if ( distance.squaredNorm() <= inlier_distance*inlier_distance )
				inlier_count++;
		}

		if ( max_inliers == 0 || inlier_count > max_inliers )
		{
			result = current_eigenvector;
			max_inliers = inlier_count;
		}
	}

	return result;
}

}
