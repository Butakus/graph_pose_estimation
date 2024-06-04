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

#include <iostream>
#include <filesystem>

#include <gpe_landmark_server/landmark_io.hpp>
#include "yaml-cpp/yaml.h"

namespace fs = std::filesystem;
using Landmark = gpe_msgs::msg::Landmark;

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

bool validate_yaml_keys(const YAML::Node & node)
{
  bool valid = true;
  if (!node["id"]) {
    std::cerr << "Landmark YAML does not have 'id' field." << std::endl;
    valid = false;
  }
  int test_int;
  if (!YAML::convert<int>::decode(node["id"], test_int)) {
    std::cerr << "Landmark 'id' field is not an integer. "
              << "Must be a positive integer." << std::endl;
    valid = false;
  }
  if (!(node["id"].as<int>() >= 0)) {
    std::cerr << "Landmark 'id' is a negative number. "
              << "Must be a positive integer." << std::endl;
    valid = false;
  }
  if (!node["type"]) {
    std::cerr << "Landmark YAML does not have 'type' field." << std::endl;
    valid = false;
  }
  if (!node["type"].IsScalar()) {
    std::cerr << "Landmark 'type' field is not a scalar value. Must be a string." << std::endl;
    valid = false;
  }
  // TODO: For now only this type is supported. Change in the future.
  if (!(node["type"].as<std::string>() == "PointXY")) {
    std::cerr << "Landmark 'type' is not PointXY. "
              << "Only PointXY type is supported." << std::endl;
    valid = false;
  }
  if (!node["coords"]) {
    std::cerr << "Landmark YAML does not have 'coords' field." << std::endl;
    valid = false;
  }
  if (!node["coords"].IsSequence()) {
    std::cerr << "Landmark 'coords' field is not a sequence. "
              << "Must be a list of numbers." << std::endl;
    valid = false;
  }
  // TODO: For now only PointXY is supported and size must be 2. Change in the future.
  if (!(node["coords"].size() == 2)) {
    std::cerr << "Landmark 'coords' size must be 2. Only PointXY type is supported." << std::endl;
    std::cerr << "Landmark " << std::endl;
    valid = false;
  }
  return valid;
}

std::map<int, Landmark> load_landmarks(const std::string & filename)
{
  // Check for empty strings
  if (filename.size() == 0) {
    throw std::invalid_argument("Empty filename");
  }

  fs::path file_path = get_file_path(filename);

  // Load YAML from file
  YAML::Node yaml_doc = YAML::LoadFile(file_path);

  // Get frame_id
  if (!yaml_doc["frame_id"]) {
    std::cerr << "YAML file does not have 'frame_id' field." << std::endl;
    throw std::invalid_argument("Landmark YAML is malformed.");
  }
  std::string frame_id = yaml_doc["frame_id"].as<std::string>();

  // Iterate over landmarks from YAML node and generate the map
  std::map<int, Landmark> landmarks;
  YAML::Node landmarks_doc = yaml_doc["landmarks"];
  for (const auto & l_node : landmarks_doc) {
    if (!validate_yaml_keys(l_node)) {
      std::cerr << "Could not insert landmark. YAML file is malformed." << std::endl;
      throw std::invalid_argument("Landmark YAML is malformed.");
    }
    Landmark l;
    l.frame_id = frame_id;
    l.id = l_node["id"].as<int>();
    l.x = l_node["coords"][0].as<double>();
    l.y = l_node["coords"][1].as<double>();
    bool insert_ok = landmarks.insert({l.id, l}).second;
    if (!insert_ok) {
      std::cerr << "Could not insert landmark. "
                << "YAML file has a duplicate ID: " << l.id << std::endl;
      throw std::invalid_argument("Landmark YAML has duplicate ID");
    }
  }

  return landmarks;
}

}  // namespace gpe
