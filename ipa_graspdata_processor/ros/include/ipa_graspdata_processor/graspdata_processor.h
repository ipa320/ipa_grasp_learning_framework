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

#ifndef GRASPDATA_SAVER_H
#define GRASPDATA_SAVER_H


#include <ros/ros.h>
#include <ipa_grasppose_srvs/ProcessGraspData.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <boost/thread/mutex.hpp>
#include <visualization_msgs/MarkerArray.h>


class GraspdataProcessor
{
protected:
	ros::Subscriber camera_info_subscriber_;
	cv::Mat intrinsic_matrix_;
	cv::Size depth_path_resolution_;
	boost::mutex intrinsic_matrix_mutex_;
	size_t processed_data_counter_;  // counts processed grasp poses
	size_t processed_depth_images_;  // counts processed depth images -> used for visualization
	bool visualize_data_;
	int raw_data_visualization_step_;  // step through raw data using this step size
	int processed_data_visualization_step_;  // step through processed data using this step size
	bool initialized_;

	// Publishers for visualization
	ros::Publisher depth_image_pub_;
	ros::Publisher point_cloud_2_pub_;
	ros::Publisher grasp_poses_pub_;
	ros::Publisher depth_patch_pub_;
	ros::Publisher point_cloud_2_patch_pub_;
	ros::Publisher grasp_poses_normalized_pub_;

	// DEBUGGING
	cv::Mat depth_patch_view_;
	// End debugging

	bool convertImageMessageToCVMat(const sensor_msgs::Image &image_msg, cv::Mat &image_out);
	void createDepthPatchMsg(const geometry_msgs::Point &cuboid_center, const geometry_msgs::Vector3 &side_lengths, const cv::Mat &intrinsics, const cv::Mat &depth_image,
							 const std_msgs::Header &header, const std::string image_encoding, sensor_msgs::Image &depth_patch_msg);
	void buildCropRectanglePixel(const cv::Vec3d &cuboid_center, const cv::MatSize &depth_image_size, const cv::Mat &intrinsics, const double &width, const double &height, cv::Rect &rectangle);
	void normalizeDepthValues(const cv::Vec3d &cuboid_center, const double &depth, cv::Mat &depth_patch);
	void normalizeGraspPoint(const geometry_msgs::Point &grasp_point, const double &depth, const cv::Vec3d &cuboid_center, geometry_msgs::Point &normalized_Point);


public:

	GraspdataProcessor(ros::NodeHandle &nh);
	bool processGraspData(ipa_grasppose_srvs::ProcessGraspData::Request &request, ipa_grasppose_srvs::ProcessGraspData::Response &response);
	void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &camera_info_msg);

};

#endif // GRASPDATA_PROCESSOR_H
