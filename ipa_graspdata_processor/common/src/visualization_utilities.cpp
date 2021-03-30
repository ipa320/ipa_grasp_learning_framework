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

#include <ipa_graspdata_processor/visualization_utilities.h>
#include <tf/LinearMath/Matrix3x3.h>

namespace VisUtils
{

void convertToPointCloud2(const cv::Mat &depth_img, const cv::Mat &intrinsics, sensor_msgs::PointCloud2 &point_cloud2)
{
	// Set some meta data of the point cloud 2
	const unsigned int &height = depth_img.rows;
	const unsigned int &width = depth_img.cols;
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

	const double xscale = 1.0/intrinsics.at<double>(0);
	const double yscale = 1.0/intrinsics.at<double>(4);
	const double xshift = -(double)width*0.5;
	const double yshift = -(double)height*0.5;

	// 2. Extract rgb and depth information from raw data
	size_t data_offset = 0;
	for ( unsigned int i=0; i<height; ++i )
	{
		for ( unsigned int j=0; j<width; ++j )
		{
			const double z = depth_img.at<float>(i, j);
			const double x = (j+xshift)*z*xscale;
			const double y = (i+yshift)*z*yscale;

			// store position to point cloud 2 data storage
			*(float*)&point_cloud2.data[data_offset] = x;  // cast address of data element to float* and then dereference it to store value to it
			data_offset += 4;
			*(float*)&point_cloud2.data[data_offset] = y;
			data_offset += 4;
			*(float*)&point_cloud2.data[data_offset] = z;
			data_offset += 4;

			// store colors to point cloud 2 data storage
			uint32_t rgb = 0xffffff;
			*(uint32_t*)&point_cloud2.data[data_offset] = rgb;
			data_offset += 4;
		}
	}
}

bool isBigEndian()
{
	const int n = 1;
	return *(char*)&n == 0;
}

void createPosesMarkerArray(const std::string &frame_id, const std::vector<geometry_msgs::Pose> &poses, visualization_msgs::MarkerArray &poses_frames)
{
	poses_frames.markers.reserve(poses.size());

	int namespace_counter = 0;
	for ( const geometry_msgs::Pose &pose : poses )
	{
		for ( short i=0; i<3; ++i )  // 3 axis -> x,y,z
		{
			visualization_msgs::Marker marker;
			marker.header.frame_id = frame_id;
			marker.header.stamp = ros::Time::now();
			marker.id = i;
			marker.ns = "poses_"+std::to_string(namespace_counter);
			marker.type = visualization_msgs::Marker::ARROW;
			marker.action = visualization_msgs::Marker::ADD;

			marker.scale.x = 0.001;
			marker.scale.y = 0.0015;
			marker.scale.z = 0.0;
			marker.color.a = 1.0;

			geometry_msgs::Point start_point;
			start_point.x = pose.position.x;
			start_point.y = pose.position.y;
			start_point.z = pose.position.z;
			marker.points.push_back(start_point);

			geometry_msgs::Point end_point;
			switch (i)
			{
				case 0:  // x-axis
					end_point.x = 0.01;
					end_point.y = 0.0;
					end_point.z = 0.0;
					marker.color.r = 1.0;
					marker.color.g = 0.0;
					marker.color.b = 0.0;
					break;
				case 1:  // y-axis
					end_point.x = 0.0;
					end_point.y = 0.01;
					end_point.z = 0.0;
					marker.color.r = 0.0;
					marker.color.g = 1.0;
					marker.color.b = 0.0;
					break;
				case 2:  // z-axis
					end_point.x = 0.0;
					end_point.y = 0.0;
					end_point.z = 0.01;
					marker.color.r = 0.0;
					marker.color.g = 0.0;
					marker.color.b = 1.0;
			}

			rotatePointByQuaternion(pose.orientation, end_point);
			end_point.x += start_point.x;
			end_point.y += start_point.y;
			end_point.z += start_point.z;
			marker.points.push_back(end_point);

			poses_frames.markers.push_back(marker);
		}

		namespace_counter++;
	}
}

void rotatePointByQuaternion(const geometry_msgs::Quaternion &rotation, geometry_msgs::Point &input_point)
{
	tf::Quaternion quat(rotation.x, rotation.y, rotation.z, rotation.w);
	tf::Matrix3x3 rotation_mat;
	rotation_mat.setRotation(quat);
	tf::Vector3 point(input_point.x, input_point.y, input_point.z);
	tf::Vector3 final_point = rotation_mat * point;
	input_point.x = final_point.x();
	input_point.y = final_point.y();
	input_point.z = final_point.z();
}

}
