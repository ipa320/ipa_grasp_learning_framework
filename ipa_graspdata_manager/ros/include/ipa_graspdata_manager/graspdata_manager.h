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

#ifndef GRASPDATA_SAVER_H
#define GRASPDATA_SAVER_H


#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <ipa_grasppose_srvs/SaveGraspData.h>
#include <opencv2/opencv.hpp>
#include <boost/thread/mutex.hpp>
#include <string>


struct ProcessedDataCounter
{
	size_t processed_data_file_counter_good;
	size_t processed_data_file_counter_bad;
	std::string associated_folder_name;

	ProcessedDataCounter(std::string folder_name) :
						processed_data_file_counter_good(0), processed_data_file_counter_bad(0), associated_folder_name(folder_name)
	{}
};

class GraspdataManager
{
protected:

	std::string raw_storage_folder_path_;
	std::string processed_storage_folder_path_;
	bool online_data_processing_;
	ros::ServiceServer save_graspposedata_service_;
	ros::ServiceClient process_graspposedata_client_;
	size_t saved_data_counter_;  // counts saved grasp poses
	int processing_max_chunk_size_;
	std::vector<ProcessedDataCounter> processed_data_file_counters_;  // needed for online processing since data might arrive unordered
	size_t *processed_data_file_counter_good_;  // points to current processed_data_file_counter_good of current processed_data_file_counters_
	size_t *processed_data_file_counter_bad_;  // points to current processed_data_file_counter_bad of current processed_data_file_counters_
	std::string previous_object_name_;  // used to reset processed_data_file_counter_ in online processing mode
	bool no_processed_data_saving_;  // avoid data saving of processed data (in case one just wants to visualize it)
	std::vector<ipa_grasppose_srvs::SaveGraspData::Request> grasp_data_queue_;
	boost::mutex grasp_data_queue_mutex_;  // mutex for multi-threaded filling of queue vector
	bool initialized_;

	bool saveDepthImage(const std::string &file_path, const std::string &file_tag, const sensor_msgs::Image &depth_image);
	bool convertImageMessageToCVMat(const sensor_msgs::Image &image_msg, cv::Mat &image_out);
	bool saveGraspData(const std::string &file_path, const std::string &file_tag, const std::vector<geometry_msgs::Pose> &grasp_poses,
					   const std::vector<uint8_t> &grasp_evaluations);
	bool saveProcessedDatasets(const std::string &storage_path, const std::string &file_name, const std::vector<sensor_msgs::Image> &depth_patches,
							   const std::vector<geometry_msgs::Pose> &grasp_poses, const std::vector<uint8_t> &grasp_evaluations);
	bool callProcessorServiceAndSaveResult(const sensor_msgs::Image &depth_image, const std::vector<geometry_msgs::Pose> &grasp_poses,
										   const std::vector<uint8_t> &grasp_evaluations, const std::string &object_name);
	bool loadDataFromRawDatabase(const std::string &folder_path);

	// Load in folders sequentially because the whole database could be very big in size to be loaded at once
	bool processDataFromFolder(const std::string &object_name, const std::vector<cv::Mat> &depth_images, const std::vector<cv::Mat> &grasp_data);

	bool createProcessedDataStorageFolders();


public:

	GraspdataManager(ros::NodeHandle &nh);
	bool addToSaveQueue(ipa_grasppose_srvs::SaveGraspData::Request &request, ipa_grasppose_srvs::SaveGraspData::Response &response);
	void saveCurrentGraspQueue();  // save current grasp queue to drive

};

#endif // GRASPDATA_SAVER_H
