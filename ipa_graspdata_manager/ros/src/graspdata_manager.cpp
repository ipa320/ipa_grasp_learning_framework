/*
 * This file is part of ipa_graspdata_manager.
 * Copyright (C) 2021  Marc Riedlinger

 * ipa_graspdata_manager is free software: you can redistribute it and/or modify
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

#include <ipa_graspdata_manager/graspdata_manager.h>
#include <ipa_graspdata_manager/file_utilities.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <ipa_grasppose_srvs/ProcessGraspData.h>


#define GOOD_FOLDER "good"
#define BAD_FOLDER "bad"


// ToDo: Move visualization code (raw and processed data separately) in here properly [Kept in processor node]
// ToDo: Add batch-wise loading of raw data since it can be too much at once [Done]
// ToDo: Better process data (with config: datasets per file), split depth pathes, grasp poses and correspondig labels -> can be read in faster afterwards by NN [Done]


GraspdataManager::GraspdataManager(ros::NodeHandle &nh) :
					saved_data_counter_(0), processed_data_file_counter_good_(nullptr), processed_data_file_counter_bad_(nullptr),
					previous_object_name_(""), no_processed_data_saving_(false), initialized_(false)
{
	nh.param<std::string>("raw_storage_folder_path", raw_storage_folder_path_, "");
	nh.param<int>("processing_max_chunk_size", processing_max_chunk_size_, 1);
	processing_max_chunk_size_ = std::max(0, processing_max_chunk_size_);  // 0 turns off usage of chunk

	bool process_raw_database = false;
	nh.param<bool>("process_raw_database", process_raw_database, false);
	if ( !process_raw_database )  // set up node so that a raw database can be created
	{
		if ( !FileUtils::checkCreateFolder(raw_storage_folder_path_) )
		{
			ROS_ERROR("GraspdataManager::GraspdataManager - Could not create storage folder for raw data.");
			return;
		}

		nh.param<bool>("online_data_processing", online_data_processing_, false);
		if ( online_data_processing_ )
		{
			process_graspposedata_client_ = nh.serviceClient<ipa_grasppose_srvs::ProcessGraspData>("/ipa_graspdata_processor/ipa_graspdata_processor/process_graspdata_srv");
			nh.param<std::string>("processed_storage_folder_path", processed_storage_folder_path_, "");
			if ( processed_storage_folder_path_.empty() )
			{
				ROS_WARN("GraspdataManager::GraspdataManager - Specified processed_storage_folder_path_ is empty, setting default path.");
				processed_storage_folder_path_ = "processed";
			}

			// Ensure that the given processed storage path does not already exist to prevent data from being overwritten
			if ( !createProcessedDataStorageFolders() )
				return;
		}

		save_graspposedata_service_ = nh.advertiseService("save_graspdata_srv", &GraspdataManager::addToSaveQueue, this);
	}
	else  // load given raw database from drive, process data and save processed data, no need for the service to be active
	{
		if ( !FileUtils::pathExists(raw_storage_folder_path_) )
		{
			ROS_ERROR("GraspdataManager::GraspdataManager - Specified raw_storage_folder_path does not exist, exiting.");
			return;
		}

		nh.param<bool>("no_processed_data_saving", no_processed_data_saving_, false);
		if ( !no_processed_data_saving_ )
		{
			nh.param<std::string>("processed_storage_folder_path", processed_storage_folder_path_, "");
			if ( processed_storage_folder_path_.empty() )
			{
				ROS_WARN("GraspdataManager::GraspdataManager - Specified processed_storage_folder_path_ is empty, setting default path.");
				processed_storage_folder_path_ = "processed";
			}

			// Ensure that the given processed storage path does not already exist to prevent data from being overwritten
			if ( !no_processed_data_saving_ && !createProcessedDataStorageFolders() )
				return;
		}

		// Set up service client and storage folder
		process_graspposedata_client_ = nh.serviceClient<ipa_grasppose_srvs::ProcessGraspData>("/ipa_graspdata_processor/ipa_graspdata_processor_offline/process_graspdata_srv");

		if ( !loadDataFromRawDatabase(raw_storage_folder_path_) )
		{
			ROS_ERROR("GraspdataManager::GraspdataManager - Could not successfully read from %s, exiting.", raw_storage_folder_path_.c_str());
			return;
		}
	}

	initialized_ = true;
}

bool GraspdataManager::createProcessedDataStorageFolders()
{
	std::string fixed_path;
	if ( !FileUtils::createUniqueFolderFrom(processed_storage_folder_path_, fixed_path) )
	{
		ROS_ERROR("GraspdataManager::createProcessedDataStorageFolders - Could not create storage folder for processed data.");
		return false;
	}
	processed_storage_folder_path_ = fixed_path;  // update processed storage folder path to fixed one
	return true;
}

bool GraspdataManager::addToSaveQueue(ipa_grasppose_srvs::SaveGraspData::Request &request, ipa_grasppose_srvs::SaveGraspData::Response &response)
{
	const size_t num_datasets = request.grasp_poses_camera_frame.size();  // each grasp pose forms with its rating an own row in a big matrix
	if ( num_datasets <= 0 || num_datasets != request.grasp_poses_evaluations.size() )
	{
		ROS_WARN("GraspdataManager::addToSaveQueue - grasp_poses_camera_frame [%d] and grasp_poses_evaluations [%d] do not have the same size or are empty.",
				 (int)num_datasets, (int)request.grasp_poses_evaluations.size());
		return false;
	}

	grasp_data_queue_mutex_.lock();
	grasp_data_queue_.push_back(request);
	grasp_data_queue_mutex_.unlock();
	return true;
}

void GraspdataManager::saveCurrentGraspQueue()
{
	if ( !initialized_ )
	{
		ROS_WARN("GraspdataManager::saveCurrentGraspQueue - Node has not been initialized.");
		return;
	}

	// Read out data from queue thread-safely and reset it
	grasp_data_queue_mutex_.lock();
	std::vector<ipa_grasppose_srvs::SaveGraspData::Request> current_queue = grasp_data_queue_;  // take a copy
	grasp_data_queue_.clear();
	grasp_data_queue_mutex_.unlock();

	for ( const ipa_grasppose_srvs::SaveGraspData::Request &request : current_queue )
	{
		const std::string storage_path = raw_storage_folder_path_+"/"+request.object_name;
		if ( !FileUtils::checkCreateFolder(storage_path) )
		{
			ROS_ERROR("GraspdataManager::saveCurrentGraspQueue - Could not create storage folder for object %s.", request.object_name.c_str());
			continue;
		}

		const std::string file_path_base = storage_path+"/"+request.object_name+"_view_"+std::to_string(request.view_counter);

		// Save raw image to drive
		if ( !saveDepthImage(file_path_base, "_depth_image", request.depth_image) )
			continue;

		// Save grasp data to drive
		if ( !saveGraspData(file_path_base, "_grasp_data", request.grasp_poses_camera_frame, request.grasp_poses_evaluations) )
			continue;

		saved_data_counter_ += request.grasp_poses_camera_frame.size();  // increment data counter

		// Process raw data and save result to drive as well
		if ( online_data_processing_ )
		{
			// Search for associated file counter
			bool found_counter = false;
			for ( ProcessedDataCounter &counter : processed_data_file_counters_ )
			{
				if ( counter.associated_folder_name.compare(request.object_name) == 0 )
				{
					processed_data_file_counter_good_ = &counter.processed_data_file_counter_good;
					processed_data_file_counter_bad_ = &counter.processed_data_file_counter_bad;
					found_counter = true;
					break;
				}
			}

			// Add new counter object
			if ( !found_counter )
			{
				ProcessedDataCounter new_counter(request.object_name);
				processed_data_file_counters_.push_back(new_counter);
				processed_data_file_counter_good_ = &processed_data_file_counters_.back().processed_data_file_counter_good;
				processed_data_file_counter_bad_ = &processed_data_file_counters_.back().processed_data_file_counter_bad;
			}

			if ( !callProcessorServiceAndSaveResult(request.depth_image, request.grasp_poses_camera_frame, request.grasp_poses_evaluations, request.object_name) )
				continue;
		}

		ROS_WARN("GraspdataManager::saveCurrentGraspQueue - Data has been saved, %d in total.", (int)saved_data_counter_);
	}
}

bool GraspdataManager::saveDepthImage(const std::string &file_path, const std::string &file_tag, const sensor_msgs::Image &depth_image)
{
	cv::Mat depth_image_mat;
	if ( !convertImageMessageToCVMat(depth_image, depth_image_mat) )
		return false;
	if ( !FileUtils::writeDepthImage(file_path+file_tag, depth_image_mat) )
		return false;

	return true;
}

bool GraspdataManager::convertImageMessageToCVMat(const sensor_msgs::Image &image_msg, cv::Mat &image_out)
{
	try
	{
		sensor_msgs::Image::ConstPtr const_image = boost::make_shared<sensor_msgs::Image>(image_msg);  // convert to const smart pointer
		const cv_bridge::CvImageConstPtr image_ptr = cv_bridge::toCvShare(const_image, sensor_msgs::image_encodings::TYPE_32FC1);
		image_out = image_ptr->image.clone();
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("GraspdataManager::convertImageMessageToCVMat - cv_bridge exception: %s", e.what());
		return false;
	}

	return true;
}

bool GraspdataManager::saveGraspData(const std::string &file_path, const std::string &file_tag, const std::vector<geometry_msgs::Pose> &grasp_poses,
									 const std::vector<uint8_t> &grasp_evaluations)
{
	const size_t rows = grasp_poses.size();
	cv::Mat grasp_data = cv::Mat::zeros(rows, 8, CV_64FC1);  // [x_pos, y_pos, z_pos, w_quat, x_quat, y_quat, z_quat, rating] defines one row
	for ( size_t i=0; i<rows; ++i )
	{
		grasp_data.at<double>(i, 0) = grasp_poses[i].position.x;
		grasp_data.at<double>(i, 1) = grasp_poses[i].position.y;
		grasp_data.at<double>(i, 2) = grasp_poses[i].position.z;
		grasp_data.at<double>(i, 3) = grasp_poses[i].orientation.w;
		grasp_data.at<double>(i, 4) = grasp_poses[i].orientation.x;
		grasp_data.at<double>(i, 5) = grasp_poses[i].orientation.y;
		grasp_data.at<double>(i, 6) = grasp_poses[i].orientation.z;
		grasp_data.at<double>(i, 7) = grasp_evaluations[i];
	}

	if ( !FileUtils::writeGraspData(file_path+file_tag, grasp_data) )
		return false;

	return true;
}

bool GraspdataManager::saveProcessedDatasets(const std::string &storage_path, const std::string &file_name, const std::vector<sensor_msgs::Image> &depth_patches,
											 const std::vector<geometry_msgs::Pose> &grasp_poses, const std::vector<uint8_t> &grasp_evaluations)
{
	if ( depth_patches.empty() || grasp_poses.empty() || grasp_evaluations.empty() )
	{
		ROS_WARN("GraspdataManager::saveProcessedDatasets - depth_patches or grasp_poses or grasp_evaluations is empty.");
		return false;
	}

	if ( grasp_poses.size() != grasp_evaluations.size() )
	{
		ROS_WARN("GraspdataManager::saveProcessedDatasets - grasp_poses and grasp_evaluations do not have the same size.");
		return false;
	}

	// Two modes: 1) One depth patch with all of its grasp poses
	//            2) Each grasp pose has its own depth patch

	if ( depth_patches.size() > 1 && depth_patches.size() != grasp_poses.size() )
	{
		ROS_WARN("GraspdataManager::saveProcessedDatasets - grasp_poses and grasp_evaluations do not have the same size.");
		return false;
	}

	// Convert raw depth images to open cv matrices
	std::vector<cv::Mat> depth_patches_mat;
	depth_patches_mat.reserve(depth_patches.size());
	for ( sensor_msgs::Image image : depth_patches )
	{
		cv::Mat patch;
		if ( !convertImageMessageToCVMat(image, patch) )
			return false;

		depth_patches_mat.push_back(patch.clone());
	}

	// Extract grasp poses and evaluations and store them to open cv matrices
	const size_t rows = grasp_poses.size();
	cv::Mat grasp_data = cv::Mat::zeros(rows, 7, CV_64FC1);  // [x_pos, y_pos, z_pos, w_quat, x_quat, y_quat, z_quat] defines one row
	cv::Mat ratings = cv::Mat::zeros(rows, 1, CV_8UC1);
	for ( size_t i=0; i<rows; ++i )
	{
		grasp_data.at<double>(i, 0) = grasp_poses[i].position.x;
		grasp_data.at<double>(i, 1) = grasp_poses[i].position.y;
		grasp_data.at<double>(i, 2) = grasp_poses[i].position.z;
		grasp_data.at<double>(i, 3) = grasp_poses[i].orientation.w;
		grasp_data.at<double>(i, 4) = grasp_poses[i].orientation.x;
		grasp_data.at<double>(i, 5) = grasp_poses[i].orientation.y;
		grasp_data.at<double>(i, 6) = grasp_poses[i].orientation.z;
		ratings.at<uint8_t>(i, 0) = grasp_evaluations[i];
	}

	for ( size_t i=0; i<rows; ++i )
	{
		const cv::Mat &grasp_dataset = grasp_data.row(i);
		const cv::Mat &grasp_rating = ratings.row(i);

		size_t *file_counter = nullptr;
		std::string final_folder;
		if ( grasp_rating.at<uint8_t>(0, 0) == 0 )
		{
			file_counter = processed_data_file_counter_bad_;
			final_folder = BAD_FOLDER;
		}
		else
		{
			file_counter = processed_data_file_counter_good_;
			final_folder = GOOD_FOLDER;
		}

		const std::string full_file_path = storage_path + "/" + final_folder + "/" + file_name;  // Decide whether current grasp goes to the good or bad folder

		if ( depth_patches_mat.size() == 1 )  // Used when use_crop_cuboid == true -> only one depth image
		{
			// one depth patch and many grasp poses/evaluations -> for fully segmented objects
			if ( !FileUtils::writeProcessedDataset(full_file_path+std::to_string(*file_counter+1), depth_patches_mat[0], grasp_dataset, grasp_rating) )
				return false;
		}
		else
		{
			// one depth and one grasp pose/evaluation -> local geometry around grasp pose
			if ( !FileUtils::writeProcessedDataset(full_file_path+std::to_string(*file_counter+1), depth_patches_mat[i], grasp_dataset, grasp_rating) )
				return false;
		}

		(*file_counter)++;
	}

	return true;
}

bool GraspdataManager::callProcessorServiceAndSaveResult(const sensor_msgs::Image &depth_image, const std::vector<geometry_msgs::Pose> &grasp_poses,
														 const std::vector<uint8_t> &grasp_evaluations, const std::string &object_name)
{
	if ( grasp_poses.empty() )
	{
		ROS_WARN("GraspdataManager::callProcessorServiceAndSaveResult - Given grasp_poses vector is empty.");
		return false;
	}

	// Implement your processing strategy in here
	ipa_grasppose_srvs::ProcessGraspData process_srv;
	process_srv.request.depth_image = depth_image;
	process_srv.request.use_crop_cuboid_center = false;
	process_srv.request.crop_cuboid_side_lengths.x = 0.2;
	process_srv.request.crop_cuboid_side_lengths.y = 0.2;
	process_srv.request.crop_cuboid_side_lengths.z = 0.2;
	/*process_srv.request.use_crop_cuboid_center = true;
	process_srv.request.crop_cuboid_center_camera_frame = grasp_poses[0].position;
	process_srv.request.crop_cuboid_side_lengths.x = 0.3;
	process_srv.request.crop_cuboid_side_lengths.y = 0.3;
	process_srv.request.crop_cuboid_side_lengths.z = 0.3;*/
	// End implementation

	// Split whole data into smaller chunks to reduce the data impact on the communication between this node and the processor node
	size_t chunk_size = process_srv.request.use_crop_cuboid_center ? 0 : processing_max_chunk_size_;  // use_crop_cuboid_center disables splitting of grasp data

	if ( chunk_size <= 0 )  // process all grasp poses in one iteration
		chunk_size = grasp_poses.size();

	size_t chunk_iterations = grasp_poses.size() / chunk_size;

	// Split the raw data to be processed into chunks of processing_max_chunk_size
	for ( size_t i=0; i<chunk_iterations+1; ++i )  // +1 to allow for processing the remainder at the end of the grasp poses vector
	{
		std::vector<geometry_msgs::Pose>::const_iterator grasp_poses_chunk_start;
		std::vector<geometry_msgs::Pose>::const_iterator grasp_poses_chunk_end;
		std::vector<uint8_t>::const_iterator grasp_evaluations_chunk_start;
		std::vector<uint8_t>::const_iterator grasp_evaluations_chunk_end;

		if ( i < chunk_iterations )  // extract chunks from grasp data
		{
			grasp_poses_chunk_start = grasp_poses.begin() + i*chunk_size;
			grasp_poses_chunk_end = grasp_poses_chunk_start + chunk_size;
			grasp_evaluations_chunk_start = grasp_evaluations.begin() + i*chunk_size;
			grasp_evaluations_chunk_end = grasp_evaluations_chunk_start + chunk_size;
		}
		else  // take care of the remaining rest that did not fit within the previous chunk iterations
		{
			const size_t rest = grasp_poses.size() % chunk_size;

			if ( rest == 0 )  // nothing to do
				break;

			grasp_poses_chunk_start = grasp_poses.end() - rest;
			grasp_poses_chunk_end = grasp_poses.end();
			grasp_evaluations_chunk_start = grasp_evaluations.end() - rest;
			grasp_evaluations_chunk_end = grasp_evaluations.end();
		}

		const std::vector<geometry_msgs::Pose> grasp_poses_chunk(grasp_poses_chunk_start, grasp_poses_chunk_end);
		const std::vector<uint8_t> grasp_evaluations_chunk(grasp_evaluations_chunk_start, grasp_evaluations_chunk_end);
		process_srv.request.grasp_poses_camera_frame = grasp_poses_chunk;

		if ( !process_graspposedata_client_.call(process_srv) )
		{
			ROS_WARN("GraspdataManager::callProcessorServiceAndSaveResult - Failed to call process_graspdata_srv service.");
			return false;
		}

		if ( process_srv.request.use_crop_cuboid_center )
		{
			if ( process_srv.response.depth_patches.size() != 1 )
			{
				ROS_WARN("GraspdataManager::callProcessorServiceAndSaveResult - Several depth_patches returned, expected 1.");
				return false;
			}
		}
		else
		{
			if ( process_srv.response.depth_patches.size() != process_srv.response.normalized_grasp_poses.size() )
			{
				ROS_WARN("GraspdataManager::callProcessorServiceAndSaveResult - normalized_grasp_poses and corresponding depth_patches do not have the same size.");
				return false;
			}
		}

		if ( process_srv.response.normalized_grasp_poses.size() != grasp_poses_chunk.size() )
		{
			ROS_WARN("GraspdataManager::callProcessorServiceAndSaveResult - Returned normalized_grasp_poses does not match with size of given grasp_poses chunk.");
			return false;
		}

		if ( !no_processed_data_saving_ )  // avoid processed data from being saved to drive
		{
			const std::string storage_path_processed_base = processed_storage_folder_path_+"/"+object_name;
			if ( !FileUtils::checkCreateFolder(storage_path_processed_base) )
			{
				ROS_ERROR("GraspdataManager::callProcessorServiceAndSaveResult - Could not create storage folder for processed data for object %s.", object_name.c_str());
				return false;
			}

			// Create folder for good grasp poses
			const std::string good_path = storage_path_processed_base+"/"+GOOD_FOLDER;
			if ( !FileUtils::checkCreateFolder(good_path) )
			{
				ROS_ERROR("GraspdataManager::callProcessorServiceAndSaveResult - Could not create storage folder for good grasps.");
				return false;
			}

			// Create folder for bad grasp poses
			const std::string bad_path = storage_path_processed_base+"/"+BAD_FOLDER;
			if ( !FileUtils::checkCreateFolder(bad_path) )
			{
				ROS_ERROR("GraspdataManager::callProcessorServiceAndSaveResult - Could not create storage folder for bad grasps.");
				return false;
			}

			const std::string file_name_base = object_name+"_dataset_";
			if ( !saveProcessedDatasets(storage_path_processed_base, file_name_base, process_srv.response.depth_patches, process_srv.response.normalized_grasp_poses,
										grasp_evaluations_chunk) )
				return false;
		}
	}

	return true;
}

bool GraspdataManager::loadDataFromRawDatabase(const std::string &folder_path)
{
	std::vector<std::string> folder_names;
	if ( !FileUtils::getSubfolders(folder_path, folder_names) )
		return false;

	bool fully_successful = true;
	for ( std::string folder : folder_names )
	{
		std::vector<cv::Mat> depth_images;
		std::vector<cv::Mat> grasp_data;
		if ( !FileUtils::readDataFromFolder(folder, "_depth_image", depth_images, grasp_data) )
		{
			ROS_WARN("GraspdataManager::loadDataFromRawDatabase - Could not read from folder [%s] -> Skipping.", folder.c_str());
			continue;
		}

		if ( depth_images.size() != grasp_data.size() )
		{
			ROS_WARN("GraspdataManager::loadDataFromRawDatabase - depth images and grasp data vectors from folder [%s] do not have the same size -> Skipping.", folder.c_str());
			continue;
		}

		std::vector<std::string> result = FileUtils::string_split(folder, "/");
		if ( result.size() <= 0 )
		{
			ROS_WARN("GraspdataManager::loadDataFromRawDatabase - Unexpected error with folder [%s], could not retrieve last subfolder's name -> Skipping.", folder.c_str());
			continue;
		}

		const std::string &object_name = result.back();  // subfolder names correspond to the object names
		fully_successful &= processDataFromFolder(object_name, depth_images, grasp_data);

		if ( !ros::ok() )  // exit in case ros is killed
			break;
	}

	if ( fully_successful )
		ROS_WARN("GraspdataManager::loadDataFromRawDatabase - Successfully processed raw database.");
	else
		ROS_WARN("GraspdataManager::loadDataFromRawDatabase - Partially processed raw database.");

	return true;
}

bool GraspdataManager::processDataFromFolder(const std::string &object_name, const std::vector<cv::Mat> &depth_images, const std::vector<cv::Mat> &grasp_data)
{
	// Convert data so that processor node can work with it
	size_t processed_data_file_counter_good = 0;  // create new counters for each new object (= folder)
	size_t processed_data_file_counter_bad = 0;
	processed_data_file_counter_good_ = &processed_data_file_counter_good;
	processed_data_file_counter_bad_ = &processed_data_file_counter_bad;
	bool successful = true;
	for ( size_t i=0; i<depth_images.size(); ++i )  // depth_images[0] corresponds to grasp_data[0]
	{
		if ( !ros::ok() )  // exit in case ros is killed
			return false;

		cv_bridge::CvImage cv_image;
		cv_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
		cv_image.image = depth_images[i];
		sensor_msgs::Image image_msg;
		cv_image.toImageMsg(image_msg);
		image_msg.header.frame_id = "Klampt_camera";
		std::vector<geometry_msgs::Pose> grasp_poses;
		std::vector<uint8_t> grasp_evaluations;
		grasp_poses.reserve(grasp_data[i].rows);
		grasp_evaluations.reserve(grasp_poses.size());
		for ( int j=0; j<grasp_data[i].rows; ++j )
		{
			geometry_msgs::Pose grasp_pose;
			grasp_pose.position.x = grasp_data[i].at<double>(j, 0);
			grasp_pose.position.y = grasp_data[i].at<double>(j, 1);
			grasp_pose.position.z = grasp_data[i].at<double>(j, 2);
			grasp_pose.orientation.w = grasp_data[i].at<double>(j, 3);
			grasp_pose.orientation.x = grasp_data[i].at<double>(j, 4);
			grasp_pose.orientation.y = grasp_data[i].at<double>(j, 5);
			grasp_pose.orientation.z = grasp_data[i].at<double>(j, 6);
			grasp_evaluations.push_back(grasp_data[i].at<double>(j, 7));
			grasp_poses.push_back(grasp_pose);
		}

		if ( !grasp_poses.empty() )  // also take into account the remainder
		{
			successful &= callProcessorServiceAndSaveResult(image_msg, grasp_poses, grasp_evaluations, object_name);
			if ( !successful )
				ROS_WARN("GraspdataManager::processDataFromFolder - Could not process some data from object %s.", object_name.c_str());
		}
	}

	return successful;
}
