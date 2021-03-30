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

#ifndef CALCULATION_UTILITIES_H
#define CALCULATION_UTILITIES_H


#include <KrisLibrary/robotics/RobotKinematics3D.h>
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math3d/AABB3D.h>
#include <Klampt/Sensing/VisualSensors.h>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>


namespace CalcUtils
{

struct RGBColor
{
	unsigned short red;		// red value [0-255]
	unsigned short green;	// green value [0-255]
	unsigned short blue;	// blue value [0-255]
};

struct HSVColor
{
	double h;  // [0,360]
	double s;  // [0,1]
	double v;  // [0,1]
};

struct PointcloudConversionInfo  // container that holds camera specific values (or points to the data)
{
	double x_shift;
	double y_shift;
	double x_scale;
	double y_scale;
	bool use_boundingbox_filter;
	bool use_color_filter;				// turns on the color filter
	double allowed_deviation_hsv[3];	// allowed deviation in each parameter h s and v
	double allowed_AABB_deviation;  // allowed deviation to AABB for a point to still be considered an object point
	Math3D::AABB3D AABB_cam;	// all points outside the bounding box in camera frame will be dropped
	RGBColor rgb_color;			// input color in rgb
	HSVColor hsv_converted;		// rbg will be converted to hsv during initialization in SimulationBackend
};

void normalizeConfiguration(Config &config, const std::vector<size_t> &revolute_indexes = std::vector<size_t>());
void normalizeAngle(double &angle);
bool forceRangeAbs(double &parameter, const double &border_abs);
bool forceRangeMinMax(double &parameter, const double &min, const double &max);
bool forceRangeMinMax(int &parameter, const int &min, const int &max);  // int version
bool equalConfigs(const Config &config_1, const Config &config_2, const Config &per_joint_accuracy_requirements,
				  const std::vector<size_t> &revolute_indexes = std::vector<size_t>());
bool equalTransforms(const Math3D::RigidTransform &transform_1, const Math3D::RigidTransform &transform_2, const double &allowed_position_deviation,
					 const double &allowed_rotation_deviation_angle);
bool checkAndFixStep(double const &start, double const &end, double &step);
void convertToFilteredPointCloud(const std::vector<double> &camera_measurements, const unsigned int &height, const unsigned int &width,
								 const PointcloudConversionInfo &pcl_conversion, std::vector<Math3D::Vector3> &point_cloud,
								 std::vector<size_t> &included_pixels);  // std::vector as Pointcloud
void convertToFilteredPointCloud2(const std::vector<double> &camera_measurements, const unsigned int &height, const unsigned int &width,
								  const PointcloudConversionInfo &pcl_conversion, sensor_msgs::PointCloud2 &point_cloud2,
								  std::vector<size_t> &included_pixels);  // ROS message of Pointcloud

bool containedInAABB(const Math3D::AABB3D &aabb, const Math3D::Vector3 &point, const double &threshold = 0.0);

void convertToPointCloud2(const std::vector<double> &camera_measurements, const unsigned int &height, const unsigned int &width,
						  const PointcloudConversionInfo &pcl_conversion, sensor_msgs::PointCloud2 &point_cloud2,
						  const std::vector<std::pair<size_t,size_t>> &original_pixel_index_pairs_ordered = std::vector<std::pair<size_t,size_t>>());
void convertToDepthImage(const std::vector<double> &camera_measurements, const unsigned int &height, const unsigned int &width, const double &depth_max,
						 sensor_msgs::Image &depth_image, const std::vector<size_t> &included_pixels = std::vector<size_t>());
bool isBigEndian();
uint32_t convertABGRToRGB(const double &abgr_value);
void convertRGBToHSV(const RGBColor &rgb_in, HSVColor &hsv_out);
bool similarColorsHSV(const HSVColor &color_1, const HSVColor &color_2, const double &allowed_deviation_h, const double &allowed_deviation_s, const double &allowed_deviation_v);

void convertToCameraInfo(const CameraSensor &camera, sensor_msgs::CameraInfo &camera_info, const bool &square_pixels = true);

void convertToPose(const Math3D::RigidTransform &transform, geometry_msgs::Pose &pose);

void getPanTiltFromSphericalCoordinates(const double &phi, const double &theta, double &pan_angle, double &tilt_angle);  // first pan, then tilt

void convertSphericalToCartesian(const double &radius, const double &phi, const double &theta, Math3D::Vector3 &cartesian);

void convertCartesianToSpherical(const Math3D::Vector3 &cartesian, const double &radius, double &phi, double &theta);  // with given radius

}

#endif // CALCULATION_UTILITIES_H
