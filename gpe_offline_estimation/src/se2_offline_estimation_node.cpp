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

#include <gpe_core/types.hpp>
#include <iostream>
#include <filesystem>
#include <gpe_core/utils.hpp>
#include <gpe_core/se2_pose_estimation.hpp>
#include <gpe_msgs/msg/landmark_detection.hpp>
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


/** Add gaussian noise to a pose.
    TODO: Allow configuring noise std as parameters.
*/
Eigen::Vector3d add_pose_noise(Eigen::Vector3d pose)
{
  pose[0] += gpe::gaussian(1.0);
  pose[1] += gpe::gaussian(1.0);
  pose[2] += gpe::gaussian(0.2);
  return pose;
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
  auto landmarks_map = gpe::load_landmarks_2d(landmarks_path);
  std::cout << "Number of landmarks: " << landmarks_map.size() << std::endl;
  for (const auto & [idx, landmark] : landmarks_map) {
    std::cout << "Landmark [" << idx << "]: " << landmark.x << ", " << landmark.y << std::endl;
  }

  // Setup pose estimator
  gpe::SE2PoseEstimation estimator;
  // Add landmarks to the graph
  for (const auto & [_, l] : landmarks_map) {
    estimator.add_landmark(l);
  }

  // Load poses
  fs::path poses_file = measurements_path / fs::path("poses.csv");  // It should be here
  std::map<int64_t, Eigen::Vector3d> gt_poses_map = gpe::parse_poses(poses_file);

  // Estimated poses to be saved later
  std::map<int64_t, Eigen::Vector3d> output_poses;

  // Load and process measurements
  using Measurements = std::map<int, gpe_msgs::msg::LandmarkDetection>;
  auto measurement_files = gpe::find_measurement_files(measurements_path);
  std::cout << "Number of measurement CSV files: " << measurement_files.size() << std::endl;
  for (const auto & [timestamp, measurement_file] : measurement_files) {
    std::cout << "##########################################" << std::endl;
    std::cout << "Processing file with timestamp: " << timestamp << std::endl;
    // First, find if there is a pose with this timestamp
    if (!gt_poses_map.contains(timestamp)) {
      std::cerr << "WARNING: Could not find a pose for timestamp: " << timestamp
                << ". Skipping measurements" << std::endl;
      continue;
    }
    // Get measurements and their corresponding pose
    Measurements measurements = gpe::parse_measurements(measurement_file);
    std::cout << "Landmarks detected: " << measurements.size() << std::endl;
    Eigen::Vector3d pose = gt_poses_map[timestamp];
    std::cout << "Ground truth pose:\n" << pose << std::endl;

    // Set initial estimation and measurements
    Eigen::Vector3d noisy_pose = add_pose_noise(pose);
    std::cout << "Initial noisy pose:\n" << noisy_pose << std::endl;
    estimator.set_initial_pose(noisy_pose);

    // TODO: This only works for LiDAR and XY landmark detections
    for (const auto & [idx, m] : measurements) {
      Eigen::Matrix2d inf_matrix;
      for (size_t i = 0; i < m.covariance.size(); i++) {
        inf_matrix(i) = m.covariance[i];
      }
      inf_matrix = inf_matrix.inverse();
      gpe::MeasurementXY measurement({m.x, m.y}, inf_matrix);
      estimator.add_measurement(measurement, idx);
    }

    // Estimate pose
    g2o::SE2 robot_pose = estimator.estimate();
    std::cout << "Estimated pose: "
              << robot_pose.translation().x() << ", "
              << robot_pose.translation().y() << " | "
              << robot_pose.rotation().angle() << std::endl;
    // Save pose to vector
    output_poses.insert({timestamp, robot_pose.toVector()});
  }

  // Save all estimated poses to output file
  // TODO: Allow output file path to be set from commandline arguments
  fs::path output_poses_file = measurements_path / fs::path("output_poses.csv");
  gpe::write_poses(output_poses, output_poses_file);

  return 0;
}
