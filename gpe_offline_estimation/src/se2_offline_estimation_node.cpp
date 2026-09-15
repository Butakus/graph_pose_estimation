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
#include <gpe_msgs/msg/landmark_detection2_d.hpp>
#include <gpe_landmark_server/landmark_io.hpp>
#include <rcutils/cmdline_parser.h>
#include <gpe_offline_estimation/file_io.hpp>
#include <string>
#include <vector>

namespace fs = std::filesystem;

void print_usage()
{
  std::cout << "Usage: se2_offline_estimation "
            << "[landmarks_file] [detections_dir] [--kabsch | --pose-avg] [--output output_file]"
            << std::endl;
}

enum class EstimationMethod
{
  G2O,
  KABSCH,
  POSE_AVG
};

/// Collect positional arguments, skipping known flags/options and their values.
std::vector<std::string> get_positional_args(int argc, char ** argv)
{
  std::vector<std::string> positional_args;
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--kabsch" || arg == "--pose-avg" || arg == "-h") {
      continue;
    }
    if (arg == "--output") {
      i++;  // Skip the option's value too
      continue;
    }
    positional_args.push_back(arg);
  }
  return positional_args;
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

  std::vector<std::string> positional_args = get_positional_args(argc, argv);
  if (positional_args.size() < 2) {
    std::cerr << "ERROR: Missing landmarks_file and/or detections_dir arguments" << std::endl;
    print_usage();
    return 1;
  }
  fs::path landmarks_path = positional_args[0];
  fs::path measurements_path = positional_args[1];

  std::cout << "Path to landmarks: " << landmarks_path << std::endl;
  std::cout << "Path to measurements: " << measurements_path << std::endl;

  // Select estimation method (defaults to g2o-based estimation)
  EstimationMethod method = EstimationMethod::G2O;
  bool kabsch_option = rcutils_cli_option_exist(argv, argv + argc, "--kabsch");
  bool pose_avg_option = rcutils_cli_option_exist(argv, argv + argc, "--pose-avg");
  if (kabsch_option && pose_avg_option) {
    std::cerr << "ERROR: --kabsch and --pose-avg are mutually exclusive" << std::endl;
    return 1;
  } else if (kabsch_option) {
    method = EstimationMethod::KABSCH;
  } else if (pose_avg_option) {
    method = EstimationMethod::POSE_AVG;
  }

  // Output file path (defaults to output_poses.csv inside the measurements directory)
  fs::path output_poses_file = measurements_path / fs::path("output_poses.csv");
  if (rcutils_cli_option_exist(argv, argv + argc, "--output")) {
    output_poses_file = rcutils_cli_get_option(argv, argv + argc, "--output");
  }

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
  using Measurements = std::map<int, gpe_msgs::msg::LandmarkDetection2D>;
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

    for (const auto & [idx, m] : measurements) {
      if (m.landmark.type == gpe_msgs::msg::Landmark2D::TYPE_XY) {
        Eigen::Matrix2d cov_matrix;
        cov_matrix(0, 0) = m.covariance[0];
        cov_matrix(0, 1) = m.covariance[1];
        cov_matrix(1, 0) = m.covariance[3];
        cov_matrix(1, 1) = m.covariance[4];
        gpe::MeasurementXY measurement({m.landmark.x, m.landmark.y}, cov_matrix.inverse());
        estimator.add_measurement(measurement, idx);
      } else if (m.landmark.type == gpe_msgs::msg::Landmark2D::TYPE_SE2) {
        Eigen::Matrix3d cov_matrix;
        for (size_t row = 0; row < 3; row++) {
          for (size_t col = 0; col < 3; col++) {
            cov_matrix(row, col) = m.covariance[row * 3 + col];
          }
        }
        gpe::MeasurementSE2 measurement(
          g2o::SE2(m.landmark.x, m.landmark.y, m.landmark.theta), cov_matrix.inverse());
        estimator.add_measurement(measurement, idx);
      }
    }

    // Estimate pose
    g2o::SE2 robot_pose;
    switch (method) {
      case EstimationMethod::KABSCH:
        robot_pose = estimator.estimate_kabsch();
        break;
      case EstimationMethod::POSE_AVG:
        robot_pose = estimator.estimate_pose_avg();
        break;
      default:
        robot_pose = estimator.estimate();
        break;
    }
    std::cout << "Estimated pose: "
              << robot_pose.translation().x() << ", "
              << robot_pose.translation().y() << " | "
              << robot_pose.rotation().angle() << std::endl;
    // Save pose to vector
    output_poses.insert({timestamp, robot_pose.toVector()});
  }

  // Save all estimated poses to output file
  gpe::write_poses(output_poses, output_poses_file);

  return 0;
}
