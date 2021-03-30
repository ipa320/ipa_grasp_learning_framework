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

#ifndef FILE_UTILITIES_H
#define FILE_UTILITIES_H


#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace FileUtils
{

bool checkCreateFolder(const std::string &path);
bool createUniqueFolderFrom(const std::string &path, std::string &fixed_path);
bool pathExists(const std::string &file_path);
bool writeDepthImage(const std::string &file_path, const cv::Mat &depth_image);
bool writeGraspData(const std::string &file_path, const cv::Mat &grasp_data);
bool writeProcessedDataset(const std::string &file_path, const cv::Mat &depth_image, const cv::Mat &grasp_pose, const cv::Mat &grasp_label);

bool getSubfolders(const std::string &folder_path, std::vector<std::string> &subfolders);
bool readDataFromFolder(const std::string &folder_path, const std::string &depth_tag, std::vector<cv::Mat> &depth_images,
						std::vector<cv::Mat> &grasp_data);
bool sortFileVectorByViewCounter(std::string i, std::string j);
bool readDepthImage(const std::string &folder_path, cv::Mat &depth_images);
bool readGraspData(const std::string &folder_path, cv::Mat &grasp_data);
bool getFileNameOfPath(const std::string &file_path, std::string &file_name);
bool retrieveViewCounter(const std::string &file_name, size_t &view_counter);
std::vector<std::string> string_split(const std::string &source, const std::string &delimiter);

}

#endif // FILE_UTILITIES_H
