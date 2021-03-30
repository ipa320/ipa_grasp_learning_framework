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

#ifndef CALCULATION_UTILITIES_H
#define CALCULATION_UTILITIES_H


#include <geometry_msgs/Pose.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace CalcUtils
{

bool forceRangeAbs(double &parameter, const double &border_abs);
bool forceRangeMinMax(double &parameter, const double &min, const double &max);
bool checkAndFixStep(double const &start, double const &end, double &step);
Eigen::Matrix4d geometryPoseToEigenMatrix(const geometry_msgs::Pose &pose);
Eigen::AngleAxisd getAngleAxisFromVectors(const Eigen::Vector3d &vector_from, const Eigen::Vector3d &vector_to);
Eigen::Vector3d projectVectorUntilPerpendicular(const Eigen::Vector3d &project_along, const Eigen::Vector3d &project);
Eigen::Vector3d determineBestEigenvector(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, const Eigen::Matrix3d &eigenvectors, const Eigen::Vector3d &surface_point, const double &inlier_distance);

}

#endif // CALCULATION_UTILITIES_H
