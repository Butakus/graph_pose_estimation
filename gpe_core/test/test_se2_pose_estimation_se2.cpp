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

gpe::MeasurementSE2 compute_landmark_measurement(
  const g2o::SE2 & pose,
  const Landmark & landmark,
  const float noise = 0.1f
)
{
  // Compute the perfect measurement
  g2o::SE2 l_pos = {landmark.x, landmark.y, landmark.theta};
  g2o::SE2 measurement = pose.inverse() * l_pos;
  // Add gaussian noise ad fill information matrix
  measurement.setTranslation(
    measurement.translation() + Eigen::Vector2d{gpe::gaussian(noise), gpe::gaussian(noise)}
  );
  measurement.setRotation(
    Eigen::Rotation2D(measurement.rotation().angle() + gpe::gaussian(noise / 10.0f))
  );
  return gpe::MeasurementSE2(
    measurement,
    Eigen::Matrix3d::Identity() / noise
  );
}

TEST(SE2PoseEstimationTestsSE2, simple_estimation_test)
{
  gpe::SE2PoseEstimation estimator;

  // Generate test landmarks and add them to the estimator
  LandmarkArray landmarks = generate_landmarks_se2();
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
  for (const auto & l : landmarks.landmarks) {
    gpe::MeasurementSE2 measurement = compute_landmark_measurement(robot_pose_gt, l);

    estimator.add_measurement(measurement, l.id);
  }
  g2o::SE2 robot_pose = estimator.estimate();

  ASSERT_NEAR(robot_pose.translation().x(), robot_pose_gt.translation().x(), 0.2);
  ASSERT_NEAR(robot_pose.translation().y(), robot_pose_gt.translation().y(), 0.2);
  ASSERT_NEAR(robot_pose.rotation().angle(), robot_pose_gt.rotation().angle(), 0.05);
}

// Test the alternative estimation method by averaging poses (only SE2 landmarks)
TEST(SE2PoseEstimationTestsSE2, avg_pose_estimation_test)
{
  gpe::SE2PoseEstimation estimator;

  // Generate test landmarks and add them to the estimator
  LandmarkArray landmarks = generate_landmarks_se2();
  for (size_t i = 0; i < landmarks.landmarks.size(); i++) {
    landmarks.landmarks[i].id = i;
    estimator.add_landmark(landmarks.landmarks[i]);
  }

  // Create the robot pose and generate an initial estimation
  g2o::SE2 robot_pose_gt(10.0, 11.0, gpe::deg_to_rad(90.0));
  g2o::SE2 robot_pose_initial_guess(7.7, 6.0, gpe::deg_to_rad(5.0));

  // Set the initial pose estimation
  estimator.set_initial_pose(robot_pose_initial_guess);

  // Set measurements (graph edges)
  for (const auto & l : landmarks.landmarks) {
    gpe::MeasurementSE2 measurement = compute_landmark_measurement(robot_pose_gt, l, 0.01f);

    estimator.add_measurement(measurement, l.id);
  }
  g2o::SE2 robot_pose = estimator.estimate_pose_avg();

  ASSERT_NEAR(robot_pose.translation().x(), robot_pose_gt.translation().x(), 0.4);
  ASSERT_NEAR(robot_pose.translation().y(), robot_pose_gt.translation().y(), 0.4);
  ASSERT_NEAR(robot_pose.rotation().angle(), robot_pose_gt.rotation().angle(), 0.05);
}

// Add test with measurements without pre-assigned ID to test hungarian assignation
TEST(SE2PoseEstimationTestsSE2, measurement_association_test)
{
  gpe::SE2PoseEstimation estimator;

  // Generate test landmarks and add them to the estimator
  LandmarkArray landmarks = generate_landmarks_se2();
  estimator.add_landmarks(landmarks);

  // Create the robot pose and generate an initial estimation
  g2o::SE2 robot_pose_gt(10.0, 11.0, gpe::deg_to_rad(90.0));
  g2o::SE2 robot_pose_initial_guess(7.7, 6.0, gpe::deg_to_rad(65.0));

  // Set the initial pose estimation
  estimator.set_initial_pose(robot_pose_initial_guess);
  // Set measurements (graph edges)
  for (const auto & l : landmarks.landmarks) {
    gpe::MeasurementSE2 measurement = compute_landmark_measurement(robot_pose_gt, l);
    estimator.add_measurement(measurement);
  }

  g2o::SE2 robot_pose = estimator.estimate();
  ASSERT_NEAR(robot_pose.translation().x(), robot_pose_gt.translation().x(), 0.2);
  ASSERT_NEAR(robot_pose.translation().y(), robot_pose_gt.translation().y(), 0.2);
  ASSERT_NEAR(robot_pose.rotation().angle(), robot_pose_gt.rotation().angle(), 0.05);
}


// Add test with measurements without pre-assigned ID to test hungarian assignation
TEST(SE2PoseEstimationTestsSE2, measurement_association_avg_poses_test)
{
  gpe::SE2PoseEstimation estimator;

  // Generate test landmarks and add them to the estimator
  LandmarkArray landmarks = generate_landmarks_se2();
  estimator.add_landmarks(landmarks);

  // Create the robot pose and generate an initial estimation
  g2o::SE2 robot_pose_gt(10.0, 11.0, gpe::deg_to_rad(90.0));
  g2o::SE2 robot_pose_initial_guess(7.7, 6.0, gpe::deg_to_rad(65.0));

  // Set the initial pose estimation
  estimator.set_initial_pose(robot_pose_initial_guess);
  // Set measurements (graph edges)
  for (const auto & l : landmarks.landmarks) {
    gpe::MeasurementSE2 measurement = compute_landmark_measurement(robot_pose_gt, l, 0.01f);
    estimator.add_measurement(measurement);
  }

  g2o::SE2 robot_pose = estimator.estimate_pose_avg();
  ASSERT_NEAR(robot_pose.translation().x(), robot_pose_gt.translation().x(), 0.4);
  ASSERT_NEAR(robot_pose.translation().y(), robot_pose_gt.translation().y(), 0.4);
  ASSERT_NEAR(robot_pose.rotation().angle(), robot_pose_gt.rotation().angle(), 0.05);
}

// Some measurements have ID, others must be associated
TEST(SE2PoseEstimationTestsSE2, measurement_mixed_association_test)
{
  gpe::SE2PoseEstimation estimator;

  // Generate test landmarks and add them to the estimator
  LandmarkArray landmarks = generate_landmarks_se2();
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
    gpe::MeasurementSE2 measurement =
      compute_landmark_measurement(robot_pose_gt, landmarks.landmarks[i]);

    if (i % 2) {
      estimator.add_measurement(measurement);
    } else {
      estimator.add_measurement(measurement, i);
    }
  }
  g2o::SE2 robot_pose = estimator.estimate();

  ASSERT_NEAR(robot_pose.translation().x(), robot_pose_gt.translation().x(), 0.2);
  ASSERT_NEAR(robot_pose.translation().y(), robot_pose_gt.translation().y(), 0.2);
  ASSERT_NEAR(robot_pose.rotation().angle(), robot_pose_gt.rotation().angle(), 0.05);
}


TEST(SE2PoseEstimationTestsSE2, add_landmark_test)
{
  // Add two landmarks with the same ID
  gpe::SE2PoseEstimation estimator;
  Landmark l1;
  l1.type = Landmark::TYPE_XY;
  l1.id = 1;
  l1.x = 1.0;
  l1.y = 1.0;
  Landmark l2(l1);
  l2.x = 2.0;
  l2.y = 2.0;
  bool result_1 = estimator.add_landmark(l1);
  bool result_2 = estimator.add_landmark(l2);

  ASSERT_EQ(result_1, true);
  ASSERT_EQ(result_2, false);
  ASSERT_EQ(estimator.get_landmarks().landmarks.size(), 1);
}

TEST(SE2PoseEstimationTestsSE2, add_landmark_list_test)
{
  // Pass a list of IDs with different size than landmarks
  gpe::SE2PoseEstimation estimator;
  LandmarkArray landmarks = generate_landmarks_se2();

  // Pass a list of landmarks with duplicate IDs
  for (size_t i = 0; i < landmarks.landmarks.size(); i++) {
    landmarks.landmarks[i].id = i;
  }
  // Copy ID from first to last
  landmarks.landmarks.back().id = landmarks.landmarks.front().id;
  bool result = estimator.add_landmarks(landmarks);
  ASSERT_EQ(result, false);
  ASSERT_EQ(estimator.get_landmarks().landmarks.size(), 0);

  // Try again with good IDs
  for (size_t i = 0; i < landmarks.landmarks.size(); i++) {
    landmarks.landmarks[i].id = i;
  }
  estimator.add_landmarks(landmarks);
  ASSERT_EQ(estimator.get_landmarks().landmarks.size(), landmarks.landmarks.size());

  // Now add IDs where the pose_id_ should be (it should be ok)
  for (size_t i = 0; i < landmarks.landmarks.size(); i++) {
    landmarks.landmarks[i].id = 1000000 + i;
  }
  estimator.add_landmarks(landmarks);
  ASSERT_EQ(estimator.get_landmarks().landmarks.size(), 2 * landmarks.landmarks.size());
}

TEST(SE2PoseEstimationTestsSE2, add_landmark_msg_test)
{
  // Add two landmarks with the same ID
  gpe::SE2PoseEstimation estimator;
  Landmark l;
  l.id = 1;
  l.type = Landmark::TYPE_XY;
  l.x = 1.0;
  l.y = 2.0;
  bool result = estimator.add_landmark(l);

  ASSERT_EQ(result, true);
  ASSERT_EQ(estimator.get_landmarks().landmarks.size(), 1);
}

TEST(SE2PoseEstimationTestsSE2, add_landmark_array_msg_test)
{
  // Add two landmarks with the same ID
  gpe::SE2PoseEstimation estimator;
  gpe_msgs::msg::Landmark2DArray landmarks;
  for (int i = 0; i < 5; i++) {
    gpe_msgs::msg::Landmark2D l;
    l.id = i;
    l.type = Landmark::TYPE_XY;
    l.x = 2 * i;
    l.y = 2 * i + 1;
    landmarks.landmarks.push_back(l);
  }
  bool result = estimator.add_landmarks(landmarks);

  ASSERT_EQ(result, true);
  ASSERT_EQ(estimator.get_landmarks().landmarks.size(), 5);
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
