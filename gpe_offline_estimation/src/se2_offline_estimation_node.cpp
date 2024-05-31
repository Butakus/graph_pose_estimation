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
#include <gpe_core/se2_pose_estimation.hpp>
#include <gpe_msgs/msg/landmark.hpp>
#include <gpe_landmark_server/landmark_io.hpp>
#include <rcutils/cmdline_parser.h>
#include <gpe_offline_estimation/file_io.hpp>

namespace fs = std::filesystem;

void print_usage()
{
  std::cout << "Usage: se2_offline_estimation "
            << "[landmarks_file] [detections_dir]"
            << std::endl;
}


int main(int argc, char ** argv)
{
  std::cout << "SE2 Offline pose estimation" << std::endl;
  // Argument count and usage
  if (argc < 2 || rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  fs::path landmarks_path = argv[1];
  fs::path measurements_path = argv[2];

  std::cout << "Path to landmarks: " << landmarks_path << std::endl;
  std::cout << "Path to measurements: " << measurements_path << std::endl;

  // if (rcutils_cli_option_exist(argv, argv + argc, "--init")) {
  //   auto init_option = rcutils_cli_get_option(argv, argv + argc, "--init");
  //   std::cout << "Init option? -> " << init_option << std::endl;
  // }

  // Load map of landmarks from YAML file
  auto landmarks_map = gpe::load_landmarks(landmarks_path);
  std::cout << "Number of landmarks: " << landmarks_map.size() << std::endl;
  for (const auto & [idx, landmark] : landmarks_map) {
    std::cout << "Landmark [" << idx << "]: " << landmark.x << ", " << landmark.y << std::endl;
  }

  // Load measurements
  auto measurement_files = gpe::find_measurement_files(measurements_path);
  std::cout << "Number of measurement CSV files: " << measurement_files.size() << std::endl;


  return 0;
}
