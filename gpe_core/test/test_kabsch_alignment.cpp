// Copyright 2026 Francisco Miguel Moreno
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

#include <gpe_core/kabsch.hpp>
#include <gpe_core/se2_pose_estimation.hpp>
#include <gpe_core/utils.hpp>

using Landmark = gpe_msgs::msg::Landmark2D;
using LandmarkArray = gpe_msgs::msg::Landmark2DArray;

namespace
{

Eigen::Matrix<double, 2, Eigen::Dynamic> make_points(
  std::initializer_list<Eigen::Vector2d> points)
{
  Eigen::Matrix<double, 2, Eigen::Dynamic> matrix(2, static_cast<Eigen::Index>(points.size()));
  Eigen::Index column = 0;
  for (const auto & point : points) {
    matrix.col(column++) = point;
  }
  return matrix;
}

}  // namespace

LandmarkArray generate_landmarks_xy()
{
  std::vector<Eigen::Vector2d> poses
  {
    Eigen::Vector2d{5.0, 40.0},
    Eigen::Vector2d{40.0, 50.0},
    Eigen::Vector2d{55.0, 25.0},
    Eigen::Vector2d{65.0, 55.0},
    Eigen::Vector2d{85.0, 60.0}
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

gpe::MeasurementXY compute_landmark_measurement(const g2o::SE2 & pose, const Landmark & landmark)
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


TEST(KabschAlignmentTests2D, recovers_rigid_transform)
{
  const Eigen::Matrix<double, 2, Eigen::Dynamic> measured_points = make_points({
    Eigen::Vector2d{1.0, 2.0},
    Eigen::Vector2d{-1.0, 0.5},
    Eigen::Vector2d{3.0, -2.0},
    Eigen::Vector2d{0.0, 1.0},
  });

  const Eigen::Rotation2Dd rotation(gpe::deg_to_rad(90.0));
  const Eigen::Vector2d translation{4.0, -3.0};

  const Eigen::Matrix<double, 2, Eigen::Dynamic> landmark_points =
    (rotation.toRotationMatrix() * measured_points).colwise() + translation;

  const auto transformation = gpe::kabsch_alignment<2>(measured_points, landmark_points);

  Eigen::Matrix3d expected = Eigen::Matrix3d::Identity();
  expected.template block<2, 2>(0, 0) = rotation.toRotationMatrix();
  expected.template block<2, 1>(0, 2) = translation;

  EXPECT_TRUE(transformation.isApprox(expected, 1e-12));
}

TEST(KabschAlignmentTests2D, maps_measured_points_onto_landmarks)
{
  const Eigen::Matrix<double, 2, Eigen::Dynamic> measured_points = make_points({
    Eigen::Vector2d{2.0, 1.0},
    Eigen::Vector2d{3.5, -1.0},
    Eigen::Vector2d{-2.0, 4.0},
  });

  const Eigen::Rotation2Dd rotation(gpe::deg_to_rad(30.0));
  const Eigen::Vector2d translation{-5.0, 2.5};

  const Eigen::Matrix<double, 2, Eigen::Dynamic> landmark_points =
    (rotation.toRotationMatrix() * measured_points).colwise() + translation;

  const auto transformation = gpe::kabsch_alignment<2>(measured_points, landmark_points);

  for (Eigen::Index column = 0; column < measured_points.cols(); ++column) {
    const Eigen::Vector3d homogeneous_measured(
      measured_points.col(column).x(),
      measured_points.col(column).y(),
      1.0);
    const Eigen::Vector3d transformed = transformation * homogeneous_measured;
    EXPECT_TRUE(transformed.head<2>().isApprox(landmark_points.col(column), 1e-12));
  }
}

TEST(KabschAlignmentTests2D, kabsch_simple_estimation_test)
{
  gpe::SE2PoseEstimation estimator;

  // Generate test landmarks and add them to the estimator
  LandmarkArray landmarks = generate_landmarks_xy();
  for (size_t i = 0; i < landmarks.landmarks.size(); ++i) {
    landmarks.landmarks[i].id = i;
    estimator.add_landmark(landmarks.landmarks[i]);
  }

  // Create the robot pose and generate an initial estimation
  const g2o::SE2 robot_pose_gt(10.0, 11.0, gpe::deg_to_rad(90.0));
  const g2o::SE2 robot_pose_initial_guess(7.7, 6.0, gpe::deg_to_rad(65.0));

  // Set the initial pose estimation
  estimator.set_initial_pose(robot_pose_initial_guess);

  // Set measurements (graph edges)
  for (const auto & landmark : landmarks.landmarks) {
    const gpe::MeasurementXY measurement = compute_landmark_measurement(robot_pose_gt, landmark);
    estimator.add_measurement(measurement, landmark.id);
  }

  const g2o::SE2 robot_pose = estimator.estimate_kabsch();

  EXPECT_NEAR(robot_pose.translation().x(), robot_pose_gt.translation().x(), 0.2);
  EXPECT_NEAR(robot_pose.translation().y(), robot_pose_gt.translation().y(), 0.2);
  EXPECT_NEAR(robot_pose.rotation().angle(), robot_pose_gt.rotation().angle(), 0.05);
}

// Add test with measurements without pre-assigned ID to test hungarian assignation
TEST(KabschAlignmentTests2D, kabsch_measurement_association_test)
{
  gpe::SE2PoseEstimation estimator;

  // Generate test landmarks and add them to the estimator
  LandmarkArray landmarks = generate_landmarks_xy();
  estimator.add_landmarks(landmarks);

  // Create the robot pose and generate an initial estimation
  g2o::SE2 robot_pose_gt(10.0, 11.0, gpe::deg_to_rad(90.0));
  g2o::SE2 robot_pose_initial_guess(7.7, 6.0, gpe::deg_to_rad(65.0));

  // Set the initial pose estimation
  estimator.set_initial_pose(robot_pose_initial_guess);

  // Set measurements (graph edges)
  for (const auto & l : landmarks.landmarks) {
    gpe::MeasurementXY measurement = compute_landmark_measurement(robot_pose_gt, l);
    estimator.add_measurement(measurement);
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
