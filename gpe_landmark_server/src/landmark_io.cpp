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
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;
using Landmark = gpe_msgs::msg::Landmark2D;

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

bool validate_yaml_common_keys(const YAML::Node & node)
{
  bool valid = true;
  if (!node["id"]) {
    std::cerr << "Landmark YAML does not have 'id' field." << std::endl;
    valid = false;
  } else {
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
  }
  if (!node["type"]) {
    std::cerr << "Landmark YAML does not have 'type' field." << std::endl;
    valid = false;
  } else {
    if (!node["type"].IsScalar()) {
      std::cerr << "Landmark 'type' field is not a scalar value. Must be a string." << std::endl;
      valid = false;
    }
    const std::string l_type = node["type"].as<std::string>();
    if (!landmark_io::landmark_type_size.contains(l_type)) {
      std::cerr << "Landmark 'type' " << l_type
                << " is not supported" << std::endl;
      valid = false;
    }
  }
  if (!node["coords"]) {
    std::cerr << "Landmark YAML does not have 'coords' field." << std::endl;
    valid = false;
  } else {
    if (!node["coords"].IsSequence()) {
      std::cerr << "Landmark 'coords' field is not a sequence. "
                << "Must be a list of numbers." << std::endl;
      valid = false;
    }
  }
  if (node["type"] && node["coords"]) {
    const std::string l_type = node["type"].as<std::string>();
    // Check that size of coords list matches the size for the given type
    if (!(node["coords"].size() == landmark_io::landmark_type_size.at(l_type))) {
      std::cerr << "Landmark 'coords' does not match size for type " << l_type
      << " which is " << landmark_io::landmark_type_size.at(l_type) << std::endl;
      valid = false;
    }
  }

  return valid;
}

bool validate_yaml_2d_type_keys(const YAML::Node & node)
{
  bool valid = true;
  if (node ["type"]) {
    const std::string l_type = node["type"].as<std::string>();
    if (!(l_type == "PointXY" || l_type == "Pose2D")) {
      std::cerr << "Landmark 'type' is not PointXY or Pose2D. " << std::endl;
      valid = false;
    }
  } else {
    valid = false;
  }
  return valid;
}

// TODO: Not supported yet
// bool validate_yaml_3d_type_keys(const YAML::Node & node)
// {
//   bool valid = true;
//   if (node ["type"]) {
//     const std::string l_type = node["type"].as<std::string>();
//     if (!(l_type == "PointXYZ" || l_type == "Pose3D" || l_type == "Pose3DQuat")) {
//       std::cerr << "Landmark 'type' is not PointXYZ or Pose3D or Pose3DQuat. " << std::endl;
//       valid = false;
//     }
//   } else {
//     valid = false;
//   }
//   return valid;
// }

std::map<int, Landmark> load_landmarks_2d(const std::string & filename)
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
    if (!validate_yaml_common_keys(l_node) || !validate_yaml_2d_type_keys(l_node)) {
      std::cerr << "Could not insert landmark. YAML file is malformed." << std::endl;
      throw std::invalid_argument("Landmark YAML is malformed.");
    }
    Landmark l;
    l.frame_id = frame_id;
    l.id = l_node["id"].as<int>();
    l.type = landmark_io::landmark_type_conversions.at(l_node["type"].as<std::string>());
    l.x = l_node["coords"][0].as<double>();
    l.y = l_node["coords"][1].as<double>();
    if (l.type == Landmark::TYPE_SE2) {
      l.theta = l_node["coords"][2].as<double>();
    }
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
