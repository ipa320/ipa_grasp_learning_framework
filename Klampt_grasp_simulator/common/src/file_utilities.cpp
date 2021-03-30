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

#include <Klampt_grasp_simulator/file_utilities.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>

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

bool writeFileContent(const std::string &file_path, const std::string &content)
{
	std::fstream file_output;
	std::string path_file_tmp = file_path + "_temp";

	// Check that _temp does not already exist! Append zeros otherwise
	unsigned short i = 0;
	while ( pathExists(path_file_tmp) )
	{
		path_file_tmp += "0";

		++i;
		if ( i >= 10 )
		{
			std::cerr << "FileUtils::checkCreateFolder - Error, could not create temporary file." << std::endl;
			return false;
		}
	}

	file_output.open(path_file_tmp.c_str(), std::ios::out | std::ios::trunc);  // Prepare temporary file for writing
	if ( !file_output.is_open() )
	{
		std::cerr << "FileUtils::checkCreateFolder - Error, could not write to temporary file " << path_file_tmp << "." << std::endl;
		return false;
	}

	file_output << content;  // Write to temporary file
	file_output.close();

	if ( pathExists(file_path) && !deleteFile(file_path) )  // Remove previous file
	{
		std::cerr << "FileUtils::checkCreateFolder - Error, could not remove old existing file " << file_path << "." << std::endl;
		return false;
	}

	if ( std::rename( path_file_tmp.c_str(), file_path.c_str() ) != 0 )  // Rename temporary file to match original file name
	{
		std::cerr << "FileUtils::checkCreateFolder - Error, could not rename temporary file " << path_file_tmp << " to " << file_path << "." << std::endl;
		return false;
	}

	return true;
}

bool readFileContent(const std::string &file_path, std::string &content)
{
	std::fstream file_output;

	content.clear();
	file_output.open(file_path.c_str(), std::ios::in);
	if ( file_output.is_open() )
	{
		content = "";

		while ( !file_output.eof() )
		{
			std::string line;
			std::getline(file_output,line);
			content += line;
			if ( !file_output.eof() )
				content += "\n";
		}

		file_output.close(); // Data has been read, now close
		return true;
	}
	else
		std::cerr << "FileUtils::checkCreateFolderError - Error, could not open " << file_path << " for reading." << std::endl;

	return false;
}

bool deleteFile(const std::string &file_path)
{
	return boost::filesystem::remove(file_path);
}

bool pathExists(const std::string &file_path)
{
	return boost::filesystem::exists(file_path);
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
