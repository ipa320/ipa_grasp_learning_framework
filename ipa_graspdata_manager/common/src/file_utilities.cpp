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

#include <ipa_graspdata_manager/file_utilities.h>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <iostream>


namespace FileUtils
{

bool checkCreateFolder(const std::string &path)
{
	if ( path.empty() )
	{
		std::cerr << "FileUtils::checkCreateFolder - Error, given path is empty." << std::endl;
		return false;
	}

	boost::filesystem::path boost_path(path);
	if ( !boost::filesystem::exists(boost_path) )
	{
		if ( !boost::filesystem::create_directories(boost_path) )
		{
			std::cerr << "FileUtils::checkCreateFolder - Error, could not create directory " << path << std::endl;
			return false;
		}
	}

	return true;
}

bool createUniqueFolderFrom(const std::string &path, std::string &fixed_path)
{
	if ( path.empty() )
	{
		std::cerr << "FileUtils::createUniqueFolderFrom - Error, given path is empty." << std::endl;
		return false;
	}

	fixed_path = path;
	if ( pathExists(path) )
	{
		std::cout << "FileUtils::createUniqueFolderFrom - Specified path " << path << " already exists, fixing..." << std::endl;
		bool found = false;
		for ( int i=2; i<10000; ++i )
		{
			std::string file_path = path + "_" + std::to_string(i);  // try again with different number attached

			if ( !FileUtils::pathExists(file_path) )
			{
				fixed_path = file_path;
				found = true;
				break;
			}
		}

		if ( !found )
		{
			std::cerr << "FileUtils::createUniqueFolderFrom - Error, could not create unique folder from " << path << "." << std::endl;
			return false;
		}
		else
			std::cout << "FileUtils::createUniqueFolderFrom - New path: " << fixed_path << "." << std::endl;
	}

	if ( !FileUtils::checkCreateFolder(fixed_path) )
	{
		std::cerr << "FileUtils::createUniqueFolderFrom - Error, could not create storage folder for processed data." << std::endl;
		return false;
	}

	return true;
}

bool pathExists(const std::string &file_path)
{
	return boost::filesystem::exists(file_path);
}

bool writeDepthImage(const std::string &file_path, const cv::Mat &depth_image)
{
	const std::string full_file_path = file_path+".yml";

	if ( depth_image.empty() )
	{
		std::cout << "FileUtils::writeDepthImage - Given depth image " << full_file_path << " is empty, not saving..." << std::endl;
		return true;
	}

	if ( pathExists(full_file_path) )
	{
		std::cout << "FileUtils::writeDepthImage - Given depth image " << full_file_path << " already exists, not saving..." << std::endl;
		return true;
	}

	cv::FileStorage file_storage(full_file_path, cv::FileStorage::WRITE);
	if ( file_storage.isOpened() )
	{
		file_storage << "depth_image" << depth_image;
		file_storage.release();
		return true;
	}
	else
		std::cerr << "FileUtils::writeDepthImage - Could not open file " << full_file_path << std::endl;

	return false;
}

bool writeGraspData(const std::string &file_path, const cv::Mat &grasp_data)
{
	const std::string full_file_path = file_path+".yml";
	cv::Mat full_grasp_data;

	if ( pathExists(full_file_path) )
	{
		cv::Mat read_data;
		cv::Mat append_data;
		cv::FileStorage file_storage(full_file_path, cv::FileStorage::READ);
		file_storage["grasp_data"] >> read_data;
		file_storage.release();

		for ( int i=0; i<grasp_data.rows; ++i )  // check that the grasps we are about to append are not containing already existing grasps
		{
			bool found_match = false;
			cv::Mat a_row = grasp_data.row(i);

			for ( int j=0; j<read_data.rows; ++j )
			{
				cv::Mat another_row = read_data.row(j);

				if ( a_row.size != another_row.size )
				{
					std::cerr << "FileUtils::writeGraspData - Error, read in grasp [" << another_row.size << "] has different size than given grasp ["
							  << a_row.size << "]!" << std::endl;
					std::cerr << "Please move or remove corrputed " << full_file_path << std::endl;
					return false;
				}

				if ( cv::countNonZero(a_row != another_row) == 0 )  // columns are equal
				{
					found_match = true;
					std::cout << "FileUtils::writeGraspData - Given grasp pose " << full_file_path << " already exists, not saving..." << std::endl;
					break;
				}
			}

			if ( !found_match )  // only add new grasps, skip existing ones
			{
				if ( append_data.empty() )
					append_data = a_row.clone();
				else
				{
					cv::Mat result;
					cv::vconcat(append_data, a_row, result);
					append_data.release();
					append_data = result.clone();
				}
			}
		}

		if ( !append_data.empty() )
			cv::vconcat(read_data, append_data, full_grasp_data);
	}
	else
		full_grasp_data = grasp_data.clone();

	if ( !full_grasp_data.empty() )
	{
		cv::FileStorage file_storage(full_file_path, cv::FileStorage::WRITE);
		if ( file_storage.isOpened() )
		{
			file_storage << "grasp_data" << full_grasp_data;
			file_storage.release();
		}
		else
		{
			std::cerr << "FileUtils::writeGraspData - Could not open file " << full_file_path << std::endl;
			return false;
		}
	}

	return true;
}

bool writeProcessedDataset(const std::string &file_path, const cv::Mat &depth_image, const cv::Mat &grasp_pose, const cv::Mat &grasp_label)
{
	const std::string full_file_path = file_path+".yml";

	if ( depth_image.empty() || grasp_pose.empty() || grasp_label.empty() )
		return false;

	if ( pathExists(full_file_path) )
	{
		std::cout << "FileUtils::writeDataset - Given dataset " << full_file_path << " already exists, not saving..." << std::endl;
		return true;
	}

	cv::FileStorage file_storage(full_file_path, cv::FileStorage::WRITE);
	if ( file_storage.isOpened() )
	{
		file_storage << "depth_patch" << depth_image;
		file_storage << "grasp_pose" << grasp_pose;
		file_storage << "grasp_label" << grasp_label;
		file_storage.release();
	}
	else
	{
		std::cerr << "FileUtils::writeDataset - Could not open file " << full_file_path << std::endl;
		return false;
	}

	return true;
}

bool getSubfolders(const std::string &folder_path, std::vector<std::string> &subfolders)
{
	boost::filesystem::path boost_path(folder_path);

	if ( !boost::filesystem::is_directory(boost_path) )
	{
		std::cerr << "FileUtils::getSubfolders - Given path [" << folder_path << "] is not a directory." << std::endl;
		return false;
	}

	for ( boost::filesystem::directory_iterator iterator(boost_path); iterator!=boost::filesystem::directory_iterator(); ++iterator )
	{
		if ( boost::filesystem::is_directory(iterator->path()) )
			subfolders.push_back(iterator->path().string());
	}

	return true;
}

bool readDataFromFolder(const std::string &folder_path, const std::string &depth_tag, std::vector<cv::Mat> &depth_images,
								 std::vector<cv::Mat> &grasp_data)
{
	boost::filesystem::path boost_path(folder_path);

	if ( !boost::filesystem::is_directory(boost_path) )
	{
		std::cerr << "FileUtils::readDataFromFolder - Given path [" << folder_path << "] is not a directory." << std::endl;
		return false;
	}

	// Load files into corresponding vectors
	std::vector<std::string> depth_image_files;
	std::vector<std::string> grasp_data_files;
	for ( boost::filesystem::directory_iterator iterator(boost_path); iterator!=boost::filesystem::directory_iterator(); ++iterator )
	{
		if ( boost::filesystem::is_regular_file(iterator->path()) )
		{
			std::string filename = iterator->path().string();

			if ( filename.find(depth_tag) != std::string::npos )
				depth_image_files.push_back(filename);
			else
				grasp_data_files.push_back(filename);
		}
	}

	// ToDo: Check here if folder name is included in files' names -> files belong to the folder.

	if ( depth_image_files.empty() && grasp_data_files.empty() )
	{
		std::cerr << "FileUtils::readDataFromFolder - Given path [" << folder_path << "] is empty." << std::endl;
		return false;
	}

	if ( depth_image_files.size() != grasp_data_files.size() )
	{
		std::cerr << "FileUtils::readDataFromFolder - Given path [" << folder_path << "] has inconsistent files (depth images size != grasp data size)." << std::endl;
		return false;
	}

	for ( size_t i=0; i<depth_image_files.size(); ++i )  // check for corrupted file names
	{
		std::string file_name = "";
		size_t view_counter = 0;
		if ( !getFileNameOfPath(depth_image_files[i], file_name) || !retrieveViewCounter(file_name, view_counter) )  // check validity of depth image file name
			return false;
		if ( !getFileNameOfPath(grasp_data_files[i], file_name) || !retrieveViewCounter(file_name, view_counter) )  // check validity of grasp data file name
			return false;
	}

	// Sort file vectors according to their view counter string -> i'th element of depth_image_files corresponds to i'th element of grasp_data_files
	std::sort(depth_image_files.begin(), depth_image_files.end(), sortFileVectorByViewCounter);
	std::sort(grasp_data_files.begin(), grasp_data_files.end(), sortFileVectorByViewCounter);

	for ( size_t i=0; i<depth_image_files.size(); ++i )  // check for missing files after sorting
	{
		std::string file_name = "";
		size_t view_counter_1 = 0;
		size_t view_counter_2 = 0;
		getFileNameOfPath(depth_image_files[i], file_name);
		retrieveViewCounter(file_name, view_counter_1);
		getFileNameOfPath(grasp_data_files[i], file_name);
		retrieveViewCounter(file_name, view_counter_2);

		if ( view_counter_1 != view_counter_2 )
		{
			std::cerr << "FileUtils::readDataFromFolder - Given path [" << folder_path << "] has missing files (missing depth images or grasp data for view counters "
					  << view_counter_1 << "/" << view_counter_2 << ")." << std::endl;
			return false;
		}
	}

	for ( size_t i=0; i<depth_image_files.size(); ++i )
	{
		cv::Mat depth_image;
		if ( !readDepthImage(depth_image_files[i], depth_image) )
		{
			std::cerr << "FileUtils::readDataFromFolder - Could not read depth image " << depth_image_files[i] << std::endl;
			return false;
		}

		cv::Mat gp_data;
		if ( !readGraspData(grasp_data_files[i], gp_data) )
		{
			std::cerr << "FileUtils::readDataFromFolder - Could not read grasp data " << grasp_data_files[i] << std::endl;
			return false;
		}

		depth_images.push_back(depth_image);
		grasp_data.push_back(gp_data);
	}

	return true;
}

bool sortFileVectorByViewCounter(std::string str_1, std::string str_2)
{
	std::string file_name = "";
	size_t view_1 = 0;
	getFileNameOfPath(str_1, file_name);
	retrieveViewCounter(file_name, view_1);

	size_t view_2 = 0;
	getFileNameOfPath(str_2, file_name);
	retrieveViewCounter(file_name, view_2);
	return view_1 < view_2;
}

bool readDepthImage(const std::string &file_path, cv::Mat &depth_image)
{
	cv::FileStorage file_storage(file_path, cv::FileStorage::READ);
	if ( file_storage.isOpened() )
	{
		file_storage["depth_image"] >> depth_image;
		file_storage.release();
		return true;
	}
	else
		std::cerr << "FileUtils::readDepthImage - Could not open file " << file_path << std::endl;

	return false;
}

bool readGraspData(const std::string &file_path, cv::Mat &grasp_data)
{
	cv::FileStorage file_storage(file_path, cv::FileStorage::READ);
	if ( file_storage.isOpened() )
	{
		file_storage["grasp_data"] >> grasp_data;
		file_storage.release();
		return true;
	}
	else
		std::cerr << "FileUtils::readGraspData - Could not open file " << file_path << std::endl;

	return false;
}

bool getFileNameOfPath(const std::string &file_path, std::string &file_name)
{
	if ( !boost::filesystem::is_regular_file(file_path) )
	{
		std::cerr << "FileUtils::getFileNameOfPath - Given file_path [" << file_path << "] does not include a regular file." << std::endl;
		return false;
	}

	std::vector<std::string> result = string_split(file_path, "/");

	if ( result.size() <= 0 )
	{
		std::cerr << "FileUtils::getFileNameOfPath - Unexpected error with file_path [" << file_path << "]." << std::endl;
		return false;
	}

	file_name = result[result.size()-1];  // last entry includes file name and file extension
	return true;
}

bool retrieveViewCounter(const std::string &file_name, size_t &view_counter)
{
	const std::string search_str = "view";
	std::vector<std::string> result = string_split(file_name, "_");

	for ( size_t i=0; i<result.size(); ++i )
	{
		if ( result[i].compare(search_str) == 0 )  // compare substrings until view is found, view counter should be the following entry
		{
			if ( i+1 < result.size() )
			{
				try
				{
					view_counter = std::stoul(result[i+1]);
					return true;
				}
				catch (...)
				{
					std::cerr << "FileUtils::retrieveViewCounter - Corrupted file_name [" << file_name << "] does not have a valid view counter." << std::endl;
					return false;
				}
			}
			else
			{
				std::cerr << "FileUtils::retrieveViewCounter - Corrupted file_name [" << file_name << "] does not have a view counter." << std::endl;
				return false;
			}
		}
	}

	std::cerr << "FileUtils::retrieveViewCounter - Unexpected error with file_name [" << file_name << "]." << std::endl;
	return false;
}

std::vector<std::string> string_split(const std::string &source, const std::string &delimiter)
{
	std::vector<std::string> result;
	std::string remainder = source;
	size_t position = remainder.find(delimiter);
	const size_t delimiter_size = delimiter.size();

	while ( position != std::string::npos )
	{
		if ( position > 0 )
			result.push_back(remainder.substr(0, position));

		if ( position+delimiter_size < remainder.size() )  // not at end of string
		{
			remainder = remainder.substr(position+delimiter_size, remainder.size()-delimiter_size);
			position = remainder.find(delimiter);
		}
		else  // reached end
		{
			remainder = "";
			break;
		}
	}

	if ( remainder.size() > 0 )
		result.push_back(remainder);

	return result;
}

}
