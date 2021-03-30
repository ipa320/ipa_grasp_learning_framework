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

#ifndef VISUALIZATION_UTILITIES_H
#define VISUALIZATION_UTILITIES_H


#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>


namespace VisUtils
{

void convertToPointCloud2(const cv::Mat &depth_img, const cv::Mat &intrinsics, sensor_msgs::PointCloud2 &point_cloud2);
bool isBigEndian();
void createPosesMarkerArray(const std::string &frame_id, const std::vector<geometry_msgs::Pose> &poses, visualization_msgs::MarkerArray &poses_frames);
void rotatePointByQuaternion(const geometry_msgs::Quaternion &rotation, geometry_msgs::Point &input_point);

}

#endif // VISUALIZATION_UTILITIES_H
