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

#include <gtest/gtest.h>
#include <gpe_core/se2_pose_estimation.hpp>
#include <gpe_core/utils.hpp>


static double uniform_rand(double low, double high)
{
  return low + ((double) std::rand() / (RAND_MAX + 1.0)) * (high - low);
}

static double gauss_rand(double mean, double sigma)
{
  double x, y, r2;
  do {
    x = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
    y = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
    r2 = x * x + y * y;
  } while (r2 > 1.0 || r2 == 0.0);

  return mean + sigma * y * std::sqrt(-2.0 * std::log(r2) / r2);
}

double gaussian(double sigma)
{
  return gauss_rand(0., sigma);
}

std::vector<Eigen::Vector2d> generate_landmark_points()
{
  std::vector<Eigen::Vector2d> poses
  {
    Eigen::Vector2d{5.0, 40.0},
    Eigen::Vector2d{40.0, 50.0},
    Eigen::Vector2d{55.0, 25.0},
    Eigen::Vector2d{65.0, 55.0},
    Eigen::Vector2d{85.0, 60.0}
  };

  return poses;
}

void compute_landmark_measurement(
  const g2o::SE2 & pose, const Eigen::Vector2d & landmark,
  Eigen::Vector2d & measurement, Eigen::Matrix2d & inf_matrix)
{
  // Compute the perfect measurement
  Eigen::Vector2d true_measurement = pose.inverse() * landmark;
  // Add gaussian noise
  measurement = true_measurement + Eigen::Vector2d{gaussian(0.1), gaussian(0.1)};
  // Fill the information matrix
  inf_matrix = Eigen::Matrix2d::Identity() * 10;
}

TEST(SE2PoseEstimationTests, legacy_test)
{
  gpe::SE2PoseEstimation estimator;

  // Generate test landmarks and add them to the estimator
  std::vector<Eigen::Vector2d> landmark_points = generate_landmark_points();
  for (size_t i = 0; i < landmark_points.size(); i++) {
    estimator.add_landmark(landmark_points[i], i);
  }

  // std::cout << "Landmark points (ground truth): " << std::endl;
  // for (const auto & l : estimator.get_landmarks()) {
  //   std::cout << "* " << l.transpose() << std::endl;
  // }

  // Create the robot pose and generate an initial estimation
  g2o::SE2 robot_pose_gt(10.0, 11.0, gpe::deg_to_rad(90.0));
  g2o::SE2 robot_pose_initial_guess(7.7, 6.0, gpe::deg_to_rad(65.0));
  // std::cout << "Robot pose (ground truth):" << std::endl;
  // std::cout <<
  //   robot_pose_gt.translation().transpose() << " | " <<
  //   robot_pose_gt.rotation().angle() << std::endl;
  // std::cout << "Robot pose (initial guess):" << std::endl;
  // std::cout <<
  //   robot_pose_initial_guess.translation().transpose() << " | " <<
  //   robot_pose_initial_guess.rotation().angle() << std::endl;

  // Set the initial pose estimation
  estimator.set_initial_pose(robot_pose_initial_guess);

  // Set measurements (graph edges)
  for (size_t i = 0; i < landmark_points.size(); i++) {
    Eigen::Vector2d measurement;
    Eigen::Matrix2d inf_matrix;
    compute_landmark_measurement(robot_pose_gt, landmark_points[i], measurement, inf_matrix);

    estimator.add_measurement(measurement, inf_matrix, i);
  }
  g2o::SE2 robot_pose = estimator.estimate();
  // std::cout << "robot pose (estimated):" << std::endl;
  // std::cout << robot_pose.translation().transpose() << " | "
  //           << robot_pose.rotation().angle() << std::endl;

  ASSERT_NEAR(robot_pose.translation().x(), robot_pose_gt.translation().x(), 0.2);
  ASSERT_NEAR(robot_pose.translation().y(), robot_pose_gt.translation().y(), 0.2);
  ASSERT_NEAR(robot_pose.rotation().angle(), robot_pose_gt.rotation().angle(), 0.05);
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
