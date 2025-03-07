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

#ifndef GPE_OFFLINE_ESTIMATION__FILE_IO_HPP_
#define GPE_OFFLINE_ESTIMATION__FILE_IO_HPP_

#include <iostream>
#include <fstream>
#include <map>
#include <filesystem>
#include <Eigen/Dense>
#include <gpe_msgs/msg/landmark_detection.hpp>

namespace gpe
{

/** Iterate a given directory (non-recursively) and return the path of all CSV files in it.
    Return a map where the keys are the timestamps and the values are the file paths.
*/
inline std::map<int64_t, std::filesystem::path> find_measurement_files(
  const std::filesystem::path & measurements_path)
{
  std::map<int64_t, std::filesystem::path> csv_files;
  for (const auto & entry : std::filesystem::directory_iterator(measurements_path)) {
    std::filesystem::path filepath = entry.path();
    std::string filename = entry.path().filename();
    if (
      entry.is_regular_file() &&
      filepath.extension() == ".csv" &&
      filename.substr(0, 13) == "measurements_" &&
      filename != "poses.csv")
    {
      filename = filepath.replace_extension("").filename();
      int64_t timestamp = std::stol(filename.substr(13, filename.size()));
      csv_files.insert({timestamp, entry.path()});
    }
  }
  return csv_files;
}

/** Parse the CSV file containing the robot poses and return a <timestamp, pose> map */
inline std::map<int64_t, Eigen::Vector3d> parse_poses(
  const std::filesystem::path & poses_file)
{
  std::map<int64_t, Eigen::Vector3d> poses;

  std::ifstream in_file(poses_file);
  if (!in_file.is_open()) {
    throw std::filesystem::filesystem_error("Could not open poses file", std::error_code());
  }

  std::string line;
  while (std::getline(in_file, line)) {
    std::istringstream ss(line);
    Eigen::Vector3d p;
    char sep; // To read and skip CSV separator
    int64_t timestamp;
    ss >> timestamp >> sep >> p[0] >> sep >> p[1] >> sep >> p[2];
    poses.insert({timestamp, p});
  }

  return poses;
}

/** Parse the CSV file containing the landmark measurements and return a <ID, landmark> map */
inline std::map<int, gpe_msgs::msg::LandmarkDetection> parse_measurements(
  const std::filesystem::path & measurement_file)
{
  std::map<int, gpe_msgs::msg::LandmarkDetection> measurements;

  std::ifstream in_file(measurement_file);
  if (!in_file.is_open()) {
    throw std::filesystem::filesystem_error("Could not open measurements file", std::error_code());
  }

  std::string line;
  while (std::getline(in_file, line)) {
    std::istringstream ss(line);
    gpe_msgs::msg::LandmarkDetection l;
    char sep; // To read and skip CSV separator
    ss >> l.id >> sep >> l.x >> sep >> l.y;
    for (int i = 0; i < 4; i++) {
      ss >> sep >> l.covariance[i];
    }
    measurements.insert({l.id, l});
  }

  return measurements;
}

/** Write the estimated poses to a CSV file. Output is sorted by timestamp (map keys) */
inline void write_poses(
  const std::map<int64_t, Eigen::Vector3d> & output_poses,
  const std::filesystem::path & output_poses_file)
{
  std::ofstream out_file(output_poses_file);
  if (!out_file.is_open()) {
    throw std::filesystem::filesystem_error("Could not open output poses file", std::error_code());
  }

  // TODO: Maybe consider limiting number of decimals?
  for (const auto & [timestamp, pose] : output_poses) {
    out_file << timestamp << ';'
             << pose[0] << ';'
             << pose[1] << ';'
             << pose[2] << std::endl;
  }
}

}  // namespace gpe

#endif  // GPE_OFFLINE_ESTIMATION__FILE_IO_HPP_
