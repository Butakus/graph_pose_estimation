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

using Landmark = gpe_msgs::msg::Landmark2D;
using LandmarkArray = gpe_msgs::msg::Landmark2DArray;

LandmarkArray generate_landmarks_xy()
{
  std::vector<Eigen::Vector2d> poses
  {
    Eigen::Vector2d{-5.0, -40.0},
    Eigen::Vector2d{-40.0, -50.0},
    Eigen::Vector2d{-55.0, -25.0},
    Eigen::Vector2d{-65.0, -55.0},
    Eigen::Vector2d{-85.0, -60.0}
  };

  LandmarkArray landmarks;
  for (const auto & p : poses) {
    Landmark l;
    l.type = Landmark::TYPE_XY;
    l.x = p.x();
    l.y = p.y();
    landmarks.landmarks.push_back(l);
  }

  return landmarks;
}

LandmarkArray generate_landmarks_se2()
{
  std::vector<g2o::SE2> poses
  {
    {5.0, 40.0, 0.0},
    {40.0, 50.0, gpe::deg_to_rad(90)},
    {55.0, 25.0, gpe::deg_to_rad(-90)},
    {65.0, 55.0, gpe::deg_to_rad(45)},
    {85.0, 60.0, gpe::deg_to_rad(160)},
  };

  LandmarkArray landmarks;
  for (const auto & p : poses) {
    Landmark l;
    l.type = Landmark::TYPE_SE2;
    l.x = p.translation().x();
    l.y = p.translation().y();
    l.theta = p.rotation().angle();
    landmarks.landmarks.push_back(l);
  }

  return landmarks;
}

LandmarkArray generate_landmarks()
{
  LandmarkArray landmarks = generate_landmarks_xy();
  LandmarkArray landmarks_se2 = generate_landmarks_se2();
  landmarks.landmarks.reserve(landmarks.landmarks.size() + landmarks_se2.landmarks.size());
  std::copy(
    landmarks_se2.landmarks.cbegin(),
    landmarks_se2.landmarks.cend(),
    std::back_inserter(landmarks.landmarks)
  );
  return landmarks;
}

gpe::MeasurementXY compute_landmark_measurement_xy(
  const g2o::SE2 & pose,
  const Landmark & landmark
)
{
  // Compute the perfect measurement
  Eigen::Vector2d l_pos = {landmark.x, landmark.y};
  Eigen::Vector2d true_measurement = pose.inverse() * l_pos;
  // Add gaussian noise ad fill information matrix
  constexpr float noise = 0.1f;
  return gpe::MeasurementXY(
    true_measurement + Eigen::Vector2d{gpe::gaussian(noise), gpe::gaussian(noise)},
    Eigen::Matrix2d::Identity() / noise
  );
}

gpe::MeasurementSE2 compute_landmark_measurement_se2(
  const g2o::SE2 & pose,
  const Landmark & landmark
)
{
  // Compute the perfect measurement
  g2o::SE2 l_pos = {landmark.x, landmark.y, landmark.theta};
  g2o::SE2 measurement = pose.inverse() * l_pos;
  // Add gaussian noise ad fill information matrix
  constexpr float noise = 0.1f;
  measurement.setTranslation(
    measurement.translation() + Eigen::Vector2d{gpe::gaussian(noise), gpe::gaussian(noise)}
  );
  measurement.setRotation(
    Eigen::Rotation2D(measurement.rotation().angle() + gpe::gaussian(noise))
  );
  return gpe::MeasurementSE2(
    measurement,
    Eigen::Matrix3d::Identity() / noise
  );
}

TEST(SE2PoseEstimationTestsMixedTypes, simple_estimation_test)
{
  gpe::SE2PoseEstimation estimator;

  // Generate test landmarks and add them to the estimator
  LandmarkArray landmarks = generate_landmarks();
  for (size_t i = 0; i < landmarks.landmarks.size(); i++) {
    landmarks.landmarks[i].id = i;
    estimator.add_landmark(landmarks.landmarks[i]);
  }

  ASSERT_EQ(landmarks.landmarks.size(), 10);
  ASSERT_EQ(estimator.get_landmarks().landmarks.size(), 10);

  // Create the robot pose and generate an initial estimation
  g2o::SE2 robot_pose_gt(10.0, 11.0, gpe::deg_to_rad(90.0));
  g2o::SE2 robot_pose_initial_guess(7.7, 6.0, gpe::deg_to_rad(65.0));

  // Set the initial pose estimation
  estimator.set_initial_pose(robot_pose_initial_guess);

  // Set measurements (graph edges)
  for (const auto & l : landmarks.landmarks) {
    if (l.type == Landmark::TYPE_XY) {
      gpe::MeasurementXY measurement = compute_landmark_measurement_xy(robot_pose_gt, l);
      estimator.add_measurement(measurement, l.id);
    } else if (l.type == Landmark::TYPE_SE2) {
      gpe::MeasurementSE2 measurement = compute_landmark_measurement_se2(robot_pose_gt, l);
      estimator.add_measurement(measurement, l.id);
    }
  }
  g2o::SE2 robot_pose = estimator.estimate();

  ASSERT_NEAR(robot_pose.translation().x(), robot_pose_gt.translation().x(), 0.25);
  ASSERT_NEAR(robot_pose.translation().y(), robot_pose_gt.translation().y(), 0.25);
  ASSERT_NEAR(robot_pose.rotation().angle(), robot_pose_gt.rotation().angle(), 0.05);
}

// Add test with measurements without pre-assigned ID to test hungarian assignation
TEST(SE2PoseEstimationTestsMixedTypes, measurement_association_test)
{
  gpe::SE2PoseEstimation estimator;

  // Generate test landmarks and add them to the estimator
  LandmarkArray landmarks = generate_landmarks();
  estimator.add_landmarks(landmarks);
  // Create the robot pose and generate an initial estimation
  g2o::SE2 robot_pose_gt(10.0, 11.0, gpe::deg_to_rad(90.0));
  g2o::SE2 robot_pose_initial_guess(7.7, 6.0, gpe::deg_to_rad(65.0));

  // Set the initial pose estimation
  estimator.set_initial_pose(robot_pose_initial_guess);
  // Set measurements (graph edges)
  for (const auto & l : landmarks.landmarks) {
    if (l.type == Landmark::TYPE_XY) {
      gpe::MeasurementXY measurement = compute_landmark_measurement_xy(robot_pose_gt, l);
      estimator.add_measurement(measurement);
    } else if (l.type == Landmark::TYPE_SE2) {
      gpe::MeasurementSE2 measurement = compute_landmark_measurement_se2(robot_pose_gt, l);
      estimator.add_measurement(measurement);
    }
  }

  g2o::SE2 robot_pose = estimator.estimate();

  ASSERT_NEAR(robot_pose.translation().x(), robot_pose_gt.translation().x(), 0.25);
  ASSERT_NEAR(robot_pose.translation().y(), robot_pose_gt.translation().y(), 0.25);
  ASSERT_NEAR(robot_pose.rotation().angle(), robot_pose_gt.rotation().angle(), 0.05);
}

// Some measurements have ID, others must be associated
TEST(SE2PoseEstimationTestsMixedTypes, measurement_mixed_association_test)
{
  gpe::SE2PoseEstimation estimator;

  // Generate test landmarks and add them to the estimator
  LandmarkArray landmarks = generate_landmarks();
  for (size_t i = 0; i < landmarks.landmarks.size(); i++) {
    landmarks.landmarks[i].id = i;
    estimator.add_landmark(landmarks.landmarks[i]);
  }

  // Create the robot pose and generate an initial estimation
  g2o::SE2 robot_pose_gt(10.0, 11.0, gpe::deg_to_rad(90.0));
  g2o::SE2 robot_pose_initial_guess(7.7, 6.0, gpe::deg_to_rad(65.0));

  // Set the initial pose estimation
  estimator.set_initial_pose(robot_pose_initial_guess);

  // Set measurements (graph edges)
  for (size_t i = 0; i < landmarks.landmarks.size(); i++) {
    const auto l = landmarks.landmarks[i];
    if (l.type == Landmark::TYPE_XY) {
      gpe::MeasurementXY measurement = compute_landmark_measurement_xy(robot_pose_gt, l);
      if (i % 2) {
        estimator.add_measurement(measurement);
      } else {
        estimator.add_measurement(measurement, i);
      }
    } else if (l.type == Landmark::TYPE_SE2) {
      gpe::MeasurementSE2 measurement = compute_landmark_measurement_se2(robot_pose_gt, l);
      if (i % 2) {
        estimator.add_measurement(measurement);
      } else {
        estimator.add_measurement(measurement, i);
      }
    }
  }
  g2o::SE2 robot_pose = estimator.estimate();

  ASSERT_NEAR(robot_pose.translation().x(), robot_pose_gt.translation().x(), 0.25);
  ASSERT_NEAR(robot_pose.translation().y(), robot_pose_gt.translation().y(), 0.25);
  ASSERT_NEAR(robot_pose.rotation().angle(), robot_pose_gt.rotation().angle(), 0.05);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
