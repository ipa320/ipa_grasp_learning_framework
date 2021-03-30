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

#include <ipa_graspdata_processor/graspdata_processor.h>
#include <ipa_graspdata_processor/visualization_utilities.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cmath>


GraspdataProcessor::GraspdataProcessor(ros::NodeHandle &nh) :
					processed_data_counter_(0), processed_depth_images_(0), raw_data_visualization_step_(0), processed_data_visualization_step_(0), initialized_(false)
{
	std::vector<int> depth_patch_resolution;
	nh.getParam("depth_patch_resolution", depth_patch_resolution);

	if ( depth_patch_resolution.size() != 2 )
	{
		ROS_ERROR("GraspdataProcessor::GraspdataProcessor - Invalid depth_patch_resolution size %d, expected 2.", (int)depth_patch_resolution.size());
		return;
	}

	depth_path_resolution_.width = std::abs(depth_patch_resolution[0]);  // no negative values allowed
	depth_path_resolution_.height = std::abs(depth_patch_resolution[1]);

	bool use_set_camera_intrinsics = false;
	nh.param<bool>("use_set_camera_intrinsics", use_set_camera_intrinsics, false);
	if ( use_set_camera_intrinsics )
	{
		// Load camera_intrinsics from yaml
		std::vector<double> camera_intrinsics;
		nh.getParam("camera_intrinsics", camera_intrinsics);

		if ( camera_intrinsics.size() != 9 )
		{
			ROS_ERROR("GraspdataProcessor::GraspdataProcessor - Loaded camera_intrinsics has the wrong size %d, expected 9", (int)camera_intrinsics.size());
			return;
		}

		// Map 1D vector (camera_intrinsics) to 2D matrix (intrinsic_matrix_)
		unsigned short row = 0;
		intrinsic_matrix_ = cv::Mat::zeros(3, 3, CV_64FC1);
		for ( size_t i=0; i<camera_intrinsics.size(); ++i )
		{
			unsigned short column = i % intrinsic_matrix_.cols;
			if ( i>0 && column == 0 )
				row++;

			intrinsic_matrix_.at<double>(row, column) = camera_intrinsics[i];
		}
	}
	else
		camera_info_subscriber_ = nh.subscribe<sensor_msgs::CameraInfo>("/Klampt_grasp_simulator/Klampt_grasp_simulator/Klampt_CameraInfo", 1, &GraspdataProcessor::cameraInfoCallback, this);

	nh.param<bool>("visualize_data", visualize_data_, false);
	if ( visualize_data_ )
	{
		nh.param<int>("raw_data_visualization_step", raw_data_visualization_step_, 1);
		nh.param<int>("processed_data_visualization_step", processed_data_visualization_step_, 1);
		raw_data_visualization_step_ = std::max(1, raw_data_visualization_step_);
		processed_data_visualization_step_ = std::max(1, processed_data_visualization_step_);

		depth_image_pub_ = nh.advertise<sensor_msgs::Image>("Depth_image_raw", 1, true);
		point_cloud_2_pub_ = nh.advertise<sensor_msgs::PointCloud2>("Points_registered_raw", 1, true);
		grasp_poses_pub_ = nh.advertise<visualization_msgs::MarkerArray>("Grasp_poses_raw", 1, true);
		depth_patch_pub_ = nh.advertise<sensor_msgs::Image>("Depth_image_processed", 1, true);
		point_cloud_2_patch_pub_ = nh.advertise<sensor_msgs::PointCloud2>("Points_registered_processed", 1, true);
		grasp_poses_normalized_pub_ = nh.advertise<visualization_msgs::MarkerArray>("Grasp_poses_processed", 1, true);
	}

	initialized_ = true;
}

void GraspdataProcessor::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &camera_info_msg)
{
	intrinsic_matrix_mutex_.lock();
	intrinsic_matrix_ = (cv::Mat_<double>(3,3) << camera_info_msg->K[0], camera_info_msg->K[1], camera_info_msg->K[2],
												  camera_info_msg->K[3], camera_info_msg->K[4], camera_info_msg->K[5],
												  camera_info_msg->K[6], camera_info_msg->K[7], camera_info_msg->K[8]);
	intrinsic_matrix_mutex_.unlock();
}

bool GraspdataProcessor::processGraspData(ipa_grasppose_srvs::ProcessGraspData::Request &request, ipa_grasppose_srvs::ProcessGraspData::Response &response)
{
	if ( !initialized_ )
	{
		ROS_WARN("GraspdataProcessor::processGraspData - Node has not been initialized, ignoring service call.");
		return false;
	}

	const std::string &frame_id = request.depth_image.header.frame_id;
	if ( request.crop_cuboid_side_lengths.x < 0.0001 || request.crop_cuboid_side_lengths.y < 0.0001 || request.crop_cuboid_side_lengths.z < 0.0001 )
	{
		ROS_WARN("GraspdataProcessor::processGraspData - One crop cuboid side length [%g/%g/%g] is invalid, they have to be greater than 0.0.",
				 request.crop_cuboid_side_lengths.x, request.crop_cuboid_side_lengths.y, request.crop_cuboid_side_lengths.z);
		return false;
	}

	// Check whether callback has already been called once, read intrinsic values
	// Avoid race condtion with callback, so take a copy and work with it
	intrinsic_matrix_mutex_.lock();
	if ( intrinsic_matrix_.empty() )
	{
		ROS_WARN("GraspdataProcessor::processGraspData - Intrinsic camera matrix is empty, ignoring service call.");
		return false;
	}
	const cv::Mat intrinsic_matrix_copy = intrinsic_matrix_.clone();
	intrinsic_matrix_mutex_.unlock();

	cv::Mat depth_image_mat;
	if ( !convertImageMessageToCVMat(request.depth_image, depth_image_mat) )
		return false;

	// Build the crop rectangle for each grasp pose
	response.depth_patches.reserve(request.grasp_poses_camera_frame.size());
	response.normalized_grasp_poses.reserve(request.grasp_poses_camera_frame.size());

	if ( request.use_crop_cuboid_center )  // Crop depth image at given cuboid center once
	{
		sensor_msgs::Image depth_patch_msg;
		createDepthPatchMsg(request.crop_cuboid_center_camera_frame, request.crop_cuboid_side_lengths, intrinsic_matrix_copy, depth_image_mat, request.depth_image.header,
							request.depth_image.encoding, depth_patch_msg);
		response.depth_patches.push_back(depth_patch_msg);
	}
	else
		response.depth_patches.reserve(request.grasp_poses_camera_frame.size());

	bool visualize_processed_data_ = false;
	if ( visualize_data_ && (processed_depth_images_ % raw_data_visualization_step_) == 0 )
	{
		depth_image_pub_.publish(request.depth_image);
		sensor_msgs::PointCloud2 pcl_original;
		pcl_original.header.frame_id = frame_id;
		pcl_original.header.stamp = ros::Time::now();
		VisUtils::convertToPointCloud2(depth_image_mat, intrinsic_matrix_copy, pcl_original);
		point_cloud_2_pub_.publish(pcl_original);
		visualization_msgs::MarkerArray poses_original;
		VisUtils::createPosesMarkerArray(frame_id, request.grasp_poses_camera_frame, poses_original);
		grasp_poses_pub_.publish(poses_original);
		visualize_processed_data_ = true;  // visualize processed data for this raw dataset
	}

	response.normalized_grasp_poses.reserve(request.grasp_poses_camera_frame.size());
	size_t grasp_poses_counter = 0;
	for ( geometry_msgs::Pose grasp_pose : request.grasp_poses_camera_frame )
	{
		if ( !ros::ok() )  // exit in case ros is killed
			return false;

		cv::Vec3d crop_rectangle_center;
		if ( !request.use_crop_cuboid_center )  // create local depth patches around each grasp pose
		{
			sensor_msgs::Image depth_patch_msg;
			createDepthPatchMsg(grasp_pose.position, request.crop_cuboid_side_lengths, intrinsic_matrix_copy, depth_image_mat, request.depth_image.header,
								request.depth_image.encoding, depth_patch_msg);
			response.depth_patches.push_back(depth_patch_msg);

			crop_rectangle_center[0] = grasp_pose.position.x;
			crop_rectangle_center[1] = grasp_pose.position.y;
			crop_rectangle_center[2] = grasp_pose.position.z;
		}
		else
		{
			crop_rectangle_center[0] = request.crop_cuboid_center_camera_frame.x;
			crop_rectangle_center[1] = request.crop_cuboid_center_camera_frame.y;
			crop_rectangle_center[2] = request.crop_cuboid_center_camera_frame.z;
		}

		geometry_msgs::Pose normalized_grasp_pose;
		normalizeGraspPoint(grasp_pose.position, request.crop_cuboid_side_lengths.z, crop_rectangle_center, normalized_grasp_pose.position);
		normalized_grasp_pose.orientation = grasp_pose.orientation;  // do not normalize the rotation for now
		response.normalized_grasp_poses.push_back(normalized_grasp_pose);

		if ( visualize_processed_data_ && (grasp_poses_counter % processed_data_visualization_step_) == 0 )
		{
			depth_patch_pub_.publish(response.depth_patches[response.depth_patches.size()-1]);
			sensor_msgs::PointCloud2 patch;
			patch.header.frame_id = frame_id;
			patch.header.stamp = ros::Time::now();

			VisUtils::convertToPointCloud2(depth_patch_view_, intrinsic_matrix_copy, patch);
			point_cloud_2_patch_pub_.publish(patch);

			visualization_msgs::MarkerArray poses_normalized;
			std::vector<geometry_msgs::Pose> poses_normalized_vector;  // add only one element -> the grasp pose corresponding to the depth patch
			poses_normalized_vector.push_back(normalized_grasp_pose);
			VisUtils::createPosesMarkerArray(frame_id, poses_normalized_vector, poses_normalized);
			grasp_poses_normalized_pub_.publish(poses_normalized);
			std::cout << "Press Enter to cycle through the depth patches...";
			std::cin.get();  // wait until user decides to move on
		}
		grasp_poses_counter++;
	}

	processed_data_counter_ += request.grasp_poses_camera_frame.size();
	processed_depth_images_++;
	ROS_WARN("[Service] GraspdataProcessor::processGraspData - Data has been processed, %d in total.", (int)processed_data_counter_);
	return true;
}

bool GraspdataProcessor::convertImageMessageToCVMat(const sensor_msgs::Image &image_msg, cv::Mat &image_out)
{
	try
	{
		sensor_msgs::Image::ConstPtr const_image = boost::make_shared<sensor_msgs::Image>(image_msg);  // convert to const smart pointer
		const cv_bridge::CvImageConstPtr image_ptr = cv_bridge::toCvShare(const_image, sensor_msgs::image_encodings::TYPE_32FC1);
		image_out = image_ptr->image.clone();
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("GraspdataSaver::convertImageMessageToCVMat - cv_bridge exception: %s", e.what());
		return false;
	}

	return true;
}

void GraspdataProcessor::createDepthPatchMsg(const geometry_msgs::Point &cuboid_center, const geometry_msgs::Vector3 &side_lengths, const cv::Mat &intrinsics,
											 const cv::Mat &depth_image, const std_msgs::Header &header, const std::string image_encoding, sensor_msgs::Image &depth_patch_msg)
{
	cv::Vec3d crop_rectangle_center;
	crop_rectangle_center[0] = cuboid_center.x;
	crop_rectangle_center[1] = cuboid_center.y;
	crop_rectangle_center[2] = cuboid_center.z;

	cv::Rect crop_rectangle;
	buildCropRectanglePixel(crop_rectangle_center, depth_image.size, intrinsics, side_lengths.x, side_lengths.y, crop_rectangle);
	cv::Mat depth_patch = depth_image(crop_rectangle).clone();  // crop image and perform deep copy
	normalizeDepthValues(crop_rectangle_center, side_lengths.z, depth_patch);

	cv::Mat depth_patch_resized;
	if ( depth_patch.cols > depth_path_resolution_.width && depth_patch.rows > depth_path_resolution_.height )  // downscaling
		cv::resize(depth_patch, depth_patch_resized, depth_path_resolution_, 0.0, 0.0, cv::INTER_AREA);
	else  // upscaling
		cv::resize(depth_patch, depth_patch_resized, depth_path_resolution_, 0.0, 0.0, cv::INTER_LINEAR);

	cv_bridge::CvImage depth_patch_image(header, image_encoding, depth_patch_resized);
	depth_patch_image.toImageMsg(depth_patch_msg);

	// DEBUGGING
	depth_patch_view_ = depth_patch_resized.clone();
	// End debugging
}

void GraspdataProcessor::buildCropRectanglePixel(const cv::Vec3d &cuboid_center, const cv::MatSize &depth_image_size, const cv::Mat &intrinsics, const double &width, const double &height, cv::Rect &rectangle)
{
	const double width_halved = width * 0.5;
	const double height_halved = height * 0.5;

	// Get rectangle points in camera coordinates
	std::vector<cv::Vec3d> rectangle_points;
	rectangle_points.clear();
	rectangle_points.reserve(3);
	rectangle_points.push_back(cuboid_center + cv::Vec3d(-width_halved, -height_halved, 0.0));  // top left
	rectangle_points.push_back(cuboid_center + cv::Vec3d(-width_halved, height_halved, 0.0));  // bottom left
	rectangle_points.push_back(cuboid_center + cv::Vec3d(width_halved, -height_halved, 0.0));  // top right

	const double fx = intrinsics.at<double>(0);
	const double fy = intrinsics.at<double>(4);
	const double cx = intrinsics.at<double>(2);
	const double cy = intrinsics.at<double>(5);

	// Project from camera coordinates to pixel coordinates
	std::vector<cv::Vec2i> rectangle_pixels;
	rectangle_pixels.reserve(rectangle_points.size());
	for ( cv::Vec3d point : rectangle_points )
	{
		cv::Vec2i pixel;
		pixel[0] = std::max( std::min( (int)std::round(fx * point[0]/point[2] + cx), depth_image_size[0] ), 0 );  // perspective projection formula: u = fx * X/Z + cx
		pixel[1] = std::max( std::min( (int)std::round(fy * point[1]/point[2] + cy), depth_image_size[1] ), 0 );  // v = fy * Y/Z + cy
		rectangle_pixels.push_back(pixel);
	}

	const int rectangle_width = rectangle_pixels[2][0] - rectangle_pixels[0][0];  // x-value of top right pixel minus x-value of top left
	const int rectangle_height = rectangle_pixels[1][1] - rectangle_pixels[0][1];  // y-value of bottom left pixel minus y-value of top left pixel
	rectangle.width = rectangle_width;
	rectangle.height = rectangle_height;
	rectangle.x = rectangle_pixels[0][0];  // x-value of top left
	rectangle.y = rectangle_pixels[0][1];  // y-value of top left
}

void GraspdataProcessor::normalizeDepthValues(const cv::Vec3d &cuboid_center, const double &depth, cv::Mat &depth_patch)
{
	const float depth_halved = depth * 0.5;
	const float depth_min = std::abs(cuboid_center[2] - depth_halved);
	const float depth_max = std::abs(cuboid_center[2] + depth_halved);

	if ( depth_max-depth_min <= 0.000001 )
	{
		std::cerr << "GraspdataProcessor::normalizeDepthValues - Could not normalize depth values, depth_max - depth_min is invalid." << std::endl;
		return;
	}

	// Fast and multi-core optimized foreach using a C++ Lamda
	depth_patch.forEach<float>
	(
		[&depth_min, &depth_max](float &pixel, const int* position) -> void
		{
			pixel = (pixel - depth_min) / (depth_max - depth_min);  // normalize values between [0,1]
			pixel = std::min(1.f, std::max(pixel, 0.f));  // clamp values to range [0,1]
		}
	);
}

void GraspdataProcessor::normalizeGraspPoint(const geometry_msgs::Point &grasp_point, const double &depth, const cv::Vec3d &cuboid_center, geometry_msgs::Point &normalized_Point)
{
	const cv::Vec3d gp_point(grasp_point.x, grasp_point.y, grasp_point.z);
	const cv::Vec3d center_to_grasp = gp_point - cuboid_center;
	normalized_Point.x = center_to_grasp[0];
	normalized_Point.y = center_to_grasp[1];

	const double depth_halved = depth * 0.5;
	const double depth_min = std::abs(cuboid_center[2] - depth_halved);
	const double depth_max = std::abs(cuboid_center[2] + depth_halved);

	if ( depth_max-depth_min <= 0.000001 )
		return;

	normalized_Point.z = (grasp_point.z - depth_min) / (depth_max - depth_min);  // normalize depth between [0,1]
}
