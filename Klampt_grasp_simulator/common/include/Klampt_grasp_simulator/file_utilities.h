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

#ifndef FILE_UTILITIES_H
#define FILE_UTILITIES_H


#include <string>
#include <vector>

namespace FileUtils
{

bool checkCreateFolder(const std::string &path);
bool writeFileContent(const std::string &file_path, const std::string &content); // Safely writes to file
bool readFileContent(const std::string &file_path, std::string &content);
bool deleteFile(const std::string &file_path);
bool pathExists(const std::string &file_path);

std::vector<std::string> string_split(const std::string &source, const std::string &delimiter);

}

#endif // FILE_UTILITIES_H
