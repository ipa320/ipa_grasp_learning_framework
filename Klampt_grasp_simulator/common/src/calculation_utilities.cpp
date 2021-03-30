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

#include <Klampt_grasp_simulator/calculation_utilities.h>
#include <sensor_msgs/image_encodings.h>
#include <algorithm>
#include <cmath>


namespace CalcUtils
{

void normalizeConfiguration(Config &config, const std::vector<size_t> &revolute_indexes)
{
	if ( revolute_indexes.empty() )
	{
		for ( size_t i=0; i<config.size(); ++i )
			normalizeAngle(config[i]);
	}
	else  // only normalize given subset (which indicate revolute joints), do not normalize prismatic joints
	{
		for ( size_t idx : revolute_indexes )
		{
			if ( idx < config.size() )
				normalizeAngle(config[idx]);
		}
	}
}

void normalizeAngle(double &angle)
{
	while ( angle <= -Math::Pi )
		angle += Math::TwoPi;

	while ( angle > Math::Pi )
		angle -= Math::TwoPi;

	if ( std::abs(angle) < 0.0001 )
		angle = 0;
}

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

bool forceRangeMinMax(int &parameter, const int &min, const int &max)
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

bool equalConfigs(const Config &config_1, const Config &config_2, const Config &per_joint_accuracy_requirements, const std::vector<size_t> &revolute_joint_indexes_)
{
	if ( config_1.size() != config_2.size() || config_1.size() != per_joint_accuracy_requirements.size() )
		return false;

	Config config_1_normalized = config_1;
	normalizeConfiguration(config_1_normalized, revolute_joint_indexes_);
	Config config_2_normalized = config_2;
	normalizeConfiguration(config_2_normalized, revolute_joint_indexes_);
	Config diff = config_1_normalized - config_2_normalized;
	normalizeConfiguration(diff, revolute_joint_indexes_);

	for ( size_t i=0; i<diff.size(); ++i )
	{
		if ( std::abs(diff[i]) > per_joint_accuracy_requirements[i] )
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

// Check if two transforms are equal
// First check translation of transforms and then the rotation angles
// The rotation will be compared in euler angles
bool equalTransforms(const Math3D::RigidTransform &transform_1, const Math3D::RigidTransform &transform_2, const double &allowed_position_deviation,
					 const double &allowed_rotation_deviation_angle)
{
	// Check deviation of position
	const double pos_difference = (transform_2.t - transform_1.t).norm();

	if ( pos_difference > allowed_position_deviation )
		return false;

	// Check deviation of rotation
	Math3D::EulerAngleRotation euler_angles_1;
	euler_angles_1.setMatrixZYX(transform_1.R);
	Math3D::EulerAngleRotation euler_angles_2;
	euler_angles_2.setMatrixZYX(transform_2.R);

	const double shortest_yaw = std::min( std::abs(euler_angles_1.x - euler_angles_2.x), Math::TwoPi - std::abs(euler_angles_1.x - euler_angles_2.x) );  // shortest roll angle between both rotations, absolute value
	const double shortest_pitch = std::min( std::abs(euler_angles_1.y - euler_angles_2.y), Math::TwoPi - std::abs(euler_angles_1.y - euler_angles_2.y) );
	const double shortest_roll = std::min( std::abs(euler_angles_1.z - euler_angles_2.z), Math::TwoPi - std::abs(euler_angles_1.z - euler_angles_2.z) );

	if ( shortest_roll > allowed_rotation_deviation_angle || shortest_pitch > allowed_rotation_deviation_angle || shortest_yaw > allowed_rotation_deviation_angle )
		return false;

	return true;
}

void convertToFilteredPointCloud(const std::vector<double> &camera_measurements, const unsigned int &height, const unsigned int &width,
								 const PointcloudConversionInfo &pcl_conversion, std::vector<Math3D::Vector3> &point_cloud,
								 std::vector<size_t> &included_pixels)
{

	included_pixels.clear();
	included_pixels.reserve(width*height*0.1);

	for ( unsigned int i=0; i<height; ++i )
	{
		for ( unsigned int j=0; j<width; ++j )
		{
			const size_t pixel_offset = (j+i*width);
			const double &xshift = pcl_conversion.x_shift;
			const double &yshift = pcl_conversion.y_shift;
			const double &xscale = pcl_conversion.x_scale;
			const double &yscale = pcl_conversion.y_scale;

			const double z = camera_measurements[pixel_offset + width*height];
			const double x = (j+xshift)*z*xscale;
			const double y = (i+yshift)*z*yscale;
			Math3D::Vector3 point_cloud_point(x, y, z);

			// Bounding Box filter
			if ( !containedInAABB(pcl_conversion.AABB_cam, point_cloud_point, pcl_conversion.allowed_AABB_deviation) )
				continue;

			// Color filter
			if ( pcl_conversion.use_color_filter )
			{
				const uint32_t &bgra = camera_measurements[pixel_offset];
				const uint32_t bgr = (bgra << 8) >> 8;  // set alpha channel to 0
				CalcUtils::RGBColor rgb_current;
				CalcUtils::HSVColor hsv_current;
				rgb_current.red = bgr & 0xff;
				rgb_current.green = (bgr >> 8) & 0xff;
				rgb_current.blue = (bgr >> 16) & 0xff;
				CalcUtils::convertRGBToHSV(rgb_current, hsv_current);

				if ( similarColorsHSV(hsv_current, pcl_conversion.hsv_converted, pcl_conversion.allowed_deviation_hsv[0], pcl_conversion.allowed_deviation_hsv[1], pcl_conversion.allowed_deviation_hsv[2]) )
					continue;
			}

			point_cloud.push_back(point_cloud_point);
			included_pixels.push_back(pixel_offset);
		}
	}
}

// Filters the camera measurements (from Klampt) for invalid depth data and color
void convertToFilteredPointCloud2(const std::vector<double> &camera_measurements, const unsigned int &height, const unsigned int &width,
								  const PointcloudConversionInfo &pcl_conversion, sensor_msgs::PointCloud2 &point_cloud2, std::vector<size_t> &included_pixels)
{
	std::vector<const double*> bgra_data_filtered;
	std::vector<const double*> depth_data_filtered;
	std::vector<std::pair<size_t,size_t>> original_index_pairs;  // in order to be able to calculate the correct x and y values, we need to save the original pixel coordinates which are lost after filtering

	// Allocate some elements now to save allocation time
	included_pixels.clear();
	included_pixels.reserve(width*height*0.1);
	bgra_data_filtered.reserve(width*height*0.1);
	depth_data_filtered.reserve(width*height*0.1);
	original_index_pairs.reserve(width*height*0.1);

	for ( unsigned int i=0; i<height; ++i )
	{
		for ( unsigned int j=0; j<width; ++j )
		{
			const size_t pixel_offset = (j+i*width);

			// Bounding Box filter
			if ( pcl_conversion.use_boundingbox_filter )
			{
				const double &xshift = pcl_conversion.x_shift;
				const double &yshift = pcl_conversion.y_shift;
				const double &xscale = pcl_conversion.x_scale;
				const double &yscale = pcl_conversion.y_scale;

				const double z = camera_measurements[pixel_offset + width*height];
				const double x = (j+xshift)*z*xscale;
				const double y = (i+yshift)*z*yscale;
				Math3D::Vector3 point_cloud_point(x, y, z);

				if ( !containedInAABB(pcl_conversion.AABB_cam, point_cloud_point, pcl_conversion.allowed_AABB_deviation) )
					continue;
			}

			// Color filter
			if ( pcl_conversion.use_color_filter )
			{
				const uint32_t &bgra = camera_measurements[pixel_offset];
				const uint32_t bgr = (bgra << 8) >> 8;  // set alpha channel to 0
				CalcUtils::RGBColor rgb_current;
				CalcUtils::HSVColor hsv_current;
				rgb_current.red = bgr & 0xff;
				rgb_current.green = (bgr >> 8) & 0xff;
				rgb_current.blue = (bgr >> 16) & 0xff;
				CalcUtils::convertRGBToHSV(rgb_current, hsv_current);

				if ( similarColorsHSV(hsv_current, pcl_conversion.hsv_converted, pcl_conversion.allowed_deviation_hsv[0], pcl_conversion.allowed_deviation_hsv[1], pcl_conversion.allowed_deviation_hsv[2]) )
					continue;
			}

			bgra_data_filtered.push_back(&camera_measurements[pixel_offset]);
			depth_data_filtered.push_back(&camera_measurements[pixel_offset + width*height]);
			original_index_pairs.push_back(std::pair<size_t,size_t>(j, i));  // first width
			included_pixels.push_back(pixel_offset);
		}
	}

	// Reconstruct the filtered camera measurements in the Klampt way (first bgra, followed by the depth data)
	std::vector<double> camera_measurements_filtered;
	camera_measurements_filtered.resize(bgra_data_filtered.size()*2);

	size_t depth_start = bgra_data_filtered.size();
	for ( size_t i=0; i<bgra_data_filtered.size(); ++i )
	{
		camera_measurements_filtered[i] = *bgra_data_filtered[i];
		camera_measurements_filtered[depth_start+i] = *depth_data_filtered[i];
	}

	// Due to filtering the pointcloud loses its relation to the original rectangular image structure (width and height).
	// Therefore, height has to be set to 1.
	// The function below can also be used for unfiltered pointclouds where width and height shall be preserved
	CalcUtils::convertToPointCloud2(camera_measurements_filtered, 1, bgra_data_filtered.size(), pcl_conversion, point_cloud2, original_index_pairs);
}

// Converts camera measurements (from Klampt) to the pointcloud2 format
void convertToPointCloud2(const std::vector<double> &camera_measurements, const unsigned int &height, const unsigned int &width,
						  const PointcloudConversionInfo &pcl_conversion, sensor_msgs::PointCloud2 &point_cloud2,
						  const std::vector<std::pair<size_t,size_t>> &original_pixel_index_pairs_ordered)
{
	// Set some meta data of the point cloud 2
	point_cloud2.is_bigendian = isBigEndian();
	point_cloud2.height = height;
	point_cloud2.width = width;
	point_cloud2.fields.resize(4);
	point_cloud2.fields[0].name = "x";
	point_cloud2.fields[1].name = "y";
	point_cloud2.fields[2].name = "z";
	point_cloud2.fields[3].name = "rgb";
	point_cloud2.is_dense = false;

	unsigned int offset = 0;
	for ( size_t i=0; i<point_cloud2.fields.size(); ++i, offset+=4 )
	{
		if ( i != 3 )
			point_cloud2.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
		else
			point_cloud2.fields[i].datatype = sensor_msgs::PointField::INT32;

		point_cloud2.fields[i].offset = offset;
		point_cloud2.fields[i].count = 1;
	}
	point_cloud2.point_step = offset;  // total offset for one point (x,y,z,rgb)
	point_cloud2.row_step = offset * width;
	point_cloud2.data.resize(height*width*offset);

	const double &xshift = pcl_conversion.x_shift;
	const double &yshift = pcl_conversion.y_shift;
	const double &xscale = pcl_conversion.x_scale;
	const double &yscale = pcl_conversion.y_scale;

	// 2. Extract rgb and depth information from raw data
	size_t data_offset = 0;
	for ( unsigned int i=0; i<height; ++i )
	{
		for ( unsigned int j=0; j<width; ++j )
		{
			const size_t pixel_offset = (j+i*width);

			// extract pixel position in camera frame
			const double z = camera_measurements[pixel_offset + width*height];
			double x = 0;
			double y = 0;

			if ( original_pixel_index_pairs_ordered.empty() )
			{
				x = (j+xshift)*z*xscale;
				y = (i+yshift)*z*yscale;
			}
			else  // for a filtered point cloud
			{
				x = (original_pixel_index_pairs_ordered[pixel_offset].first+xshift)*z*xscale;
				y = (original_pixel_index_pairs_ordered[pixel_offset].second+yshift)*z*yscale;
			}

			// store position to point cloud 2 data storage
			*(float*)&point_cloud2.data[data_offset] = x;  // cast address of data element to float* and then dereference it to store value to it
			data_offset += 4;
			*(float*)&point_cloud2.data[data_offset] = y;
			data_offset += 4;
			*(float*)&point_cloud2.data[data_offset] = z;
			data_offset += 4;

			// extract colors
			uint32_t rgb = convertABGRToRGB(camera_measurements[pixel_offset]);

			// store colors to point cloud 2 data storage
			*(uint32_t*)&point_cloud2.data[data_offset] = rgb;
			data_offset += 4;
		}
	}
}

bool containedInAABB(const Math3D::AABB3D &aabb, const Math3D::Vector3 &point, const double &threshold)
{
	const double allowed_theshold = std::abs(threshold);
	const Math3D::Vector3 &bmin = aabb.bmin;
	const Math3D::Vector3 &bmax = aabb.bmax;

	return (point.x-bmin.x>=-allowed_theshold && point.x-bmax.x<=allowed_theshold &&
			point.y-bmin.y>=-allowed_theshold && point.y-bmax.y<=allowed_theshold &&
			point.z-bmin.z>=-allowed_theshold && point.z-bmax.z<=allowed_theshold);
}

void convertToDepthImage(const std::vector<double> &camera_measurements, const unsigned int &height, const unsigned int &width, const double &depth_max, sensor_msgs::Image &depth_image,
						 const std::vector<size_t> &included_pixels)
{
	depth_image.is_bigendian = isBigEndian();
	depth_image.height = height;
	depth_image.width = width;
	depth_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
	depth_image.step = width*4;
	depth_image.data.resize(height*depth_image.step);

	size_t data_offset = 0;
	for ( unsigned int i=0; i<height; ++i )
	{
		for ( unsigned int j=0; j<width; ++j )
		{
			const size_t pixel_offset = (j+i*width);
			double depth = camera_measurements[pixel_offset + width*height];

			if ( !included_pixels.empty() && std::find(included_pixels.begin(), included_pixels.end(), pixel_offset) == included_pixels.end() )  // only include given pixels
				depth = depth_max;

			*(float*)&depth_image.data[data_offset] = depth;
			data_offset += 4;
		}
	}
}

bool isBigEndian()  // Basically copied from Klampt/IO/ROS.cpp
{
	const int n = 1;
	return *(char*)&n == 0;
}

uint32_t convertABGRToRGB(const double &abgr_value)  // converts abgr color representation of Klampt into rgb color representation which can be used by rviz
{
	unsigned int value = (unsigned int)abgr_value;
	unsigned int red = value & 0xff;
	unsigned int green = value & 0xff00;
	unsigned int blue = value & 0xff0000;
	//unsigned int alpha = value & 0xff000000;
	unsigned int result = (red << 16) | green | (blue >> 16);  // reconstruct color representation, dropping alpha channel
	return (uint32_t)result;
}

// Implemented as shown here: https://docs.opencv.org/3.3.0/de/d25/imgproc_color_conversions.html
void convertRGBToHSV(const RGBColor &rgb_in, HSVColor &hsv_out)
{
	const double red = rgb_in.red/255.0;
	const double green = rgb_in.green/255.0;
	const double blue = rgb_in.blue/255.0;

	const double max = std::max(red, std::max(green, blue));  // max(r,g,b)
	const double min = std::min(red, std::min(green, blue));  // min(r,g,b)
	const double delta = max - min;

	hsv_out.h = 0.0;
	hsv_out.s = 0.0;
	hsv_out.v = max;

	if ( delta > 0.0001 )
	{
		hsv_out.s = delta / max;

		if ( std::abs(red - max) < 0.0001 )  // v == red
			hsv_out.h = 60*(green-blue)/delta;
		else if ( std::abs(green - max) < 0.0001 )  // v == green
			hsv_out.h = 120 + 60*(blue-red)/delta;
		else if ( std::abs(blue - max) < 0.0001 )  // v == blue
			hsv_out.h = 240 + 60*(red-green)/delta;

		if ( hsv_out.h < 0.0 )
			hsv_out.h += 360;
	}
}

bool similarColorsHSV(const HSVColor &color_1, const HSVColor &color_2, const double &allowed_deviation_h, const double &allowed_deviation_s, const double &allowed_deviation_v)
{
	if ( std::abs(color_1.s-color_2.s) > allowed_deviation_s || std::abs(color_1.v-color_2.v) > allowed_deviation_v )
		return false;

	const double delta_hue = std::min( std::abs(color_1.h-color_2.h), 360-std::abs(color_1.h-color_2.h) );  // hue is distributed on a circle -> 0 and 360 are the same -> take smallest distance of the two
	return delta_hue <= allowed_deviation_h;
}

void convertToCameraInfo(const CameraSensor &camera, sensor_msgs::CameraInfo &camera_info, const bool &square_pixels)
{
	camera_info.width = camera.xres;
	camera_info.height = camera.yres;
	camera_info.distortion_model = "plumb_bob";
	camera_info.D.resize(5,0.0);
	double fx = 0.5*camera.xres/std::tan(camera.xfov*0.5);

	double fy = fx;
	if ( !square_pixels )
		fy = 0.5*camera.yres/std::tan(camera.yfov*0.5);

	double cx = 0.5*camera.xres;
	double cy = 0.5*camera.yres;
	camera_info.K[0] = fx;
	camera_info.K[4] = fy;
	camera_info.K[8] = 1;
	camera_info.K[2] = cx;
	camera_info.K[5] = cy;
}

void convertToPose(const Math3D::RigidTransform &transform, geometry_msgs::Pose &pose)
{
	pose.position.x = transform.t.x;
	pose.position.y = transform.t.y;
	pose.position.z = transform.t.z;

	Math3D::QuaternionRotation quaternion;
	quaternion.setMatrix(transform.R);
	quaternion.get(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
}

void getPanTiltFromSphericalCoordinates(const double &phi, const double &theta, double &pan_angle, double &tilt_angle)
{
	double phi_norm = phi;  // normalized phi
	double theta_norm = theta;

	normalizeAngle(phi_norm);
	normalizeAngle(theta_norm);

	// These calculations demand that the camera frame complies the ros standard.
	// This only works well with phi, theta within [-pi,pi]
	pan_angle = (phi >= 0.0) ? Math::Pi - phi : -Math::Pi - phi;
	tilt_angle = theta - Math::Pi_2;
}

void convertSphericalToCartesian(const double &radius, const double &phi, const double &theta, Math3D::Vector3 &cartesian)
{
	cartesian.x = radius * sin(theta) * cos(phi);
	cartesian.y = radius * sin(theta) * sin(phi);
	cartesian.z = radius * cos(theta);
}

void convertCartesianToSpherical(const Math3D::Vector3 &cartesian, const double &radius, double &phi, double &theta)
{
	const double &x = cartesian[0];
	const double &y = cartesian[1];
	const double &z = cartesian[2];

	//radius = std::sqrt(cartesian.dot(cartesian));
	theta = std::acos(z/radius);
	phi = (std::abs(x) < 0.0001 && std::abs(y) < 0.0001) ? 0.0 : std::atan2(y,x);
}

}
