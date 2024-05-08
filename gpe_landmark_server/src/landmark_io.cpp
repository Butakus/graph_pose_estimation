// Copyright 2024 Francisco Miguel Moreno
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
//  Author: Francisco Miguel Moreno

#include <filesystem>

#include <gpe_landmark_server/landmark_io.hpp>
#include "yaml-cpp/yaml.h"

namespace fs = std::filesystem;

namespace gpe
{

fs::path get_file_path(std::string filename)
{
  // First, check if the path requires expanding the home tilde '~':
  if (filename.size() > 0 && filename[0] == '~') {
    std::string home_env = std::getenv("HOME");
    filename.replace(0, 1, home_env);
  }
  // Return the absolute canonical path
  return fs::canonical(filename);
}

std::unordered_map<int, gpe_msgs::msg::Landmark> load_landmarks(const std::string & filename)
{
  // Check for empty strings
  if (filename.size() == 0) {
    throw std::invalid_argument("Empty filename");
  }

  fs::path file_path = get_file_path(filename);

  // TODO: Load YAML from file

  // TODO: Get frame_id and type

  // TODO: Iterate over landmarks from YAML node and generate the map
  std::unordered_map<int, gpe_msgs::msg::Landmark> landmarks;

  return landmarks;
}

}  // namespace gpe
