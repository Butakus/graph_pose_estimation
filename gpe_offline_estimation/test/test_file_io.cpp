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

#include <filesystem>

#include <gtest/gtest.h>
#include <gpe_landmark_server/landmark_io.hpp>
#include <gpe_offline_estimation/file_io.hpp>

namespace fs = std::filesystem;
namespace gpe
{

// Paths
const fs::path landmarks_path = TEST_DIRECTORY / fs::path("data/test_landmarks.yaml");
const fs::path measurements_path_dir = TEST_DIRECTORY / fs::path("data/measurements");
const fs::path measurements_path_dir_se2 = TEST_DIRECTORY / fs::path("data/measurements_se2");


/** Test find_measurement_files function from <gpe_offline_estimation/file_io.hpp> */
TEST(FileParsingTests, find_measurement_files)
{
  // Check if data files exist and remove them
  ASSERT_TRUE(fs::is_directory(measurements_path_dir));
  auto measurement_paths = find_measurement_files(measurements_path_dir);
  ASSERT_EQ(measurement_paths.size(), 8);
  for (const auto & [timestamp, p] : measurement_paths) {
    EXPECT_TRUE(p.extension() == ".csv");
    EXPECT_EQ(p.filename().string().rfind("measurements_", 0), 0);
  }
}

/** Test landmark loading from YAML file (using gpe_landmark_server utils) */
TEST(FileParsingTests, load_landmarks)
{
  // Load map of landmarks from YAML file
  auto landmarks_map = gpe::load_landmarks_2d(landmarks_path);
  ASSERT_EQ(landmarks_map.size(), 10);
  for (const auto &[id, l] : landmarks_map) {
    EXPECT_EQ(l.id, id);
  }
  EXPECT_EQ(landmarks_map[1].x, 16.3);
  EXPECT_EQ(landmarks_map[1].y, 16.9);
  EXPECT_EQ(landmarks_map[9].x, -8.2);
  EXPECT_EQ(landmarks_map[9].y, 6.3);
}

/** Test measurements CSV parsing
  Contents of file measurements_1717000001.csv:
  id;type;x;y;theta;covariance[0];...;covariance[8]
  1;0;-1.2940;3.1875;0.0;0.0100;0.0000;0.0000;0.0000;0.0100;0.0000;0.0000;0.0000;0.0000
  2;0;17.6870;-4.2482;0.0;0.0100;0.0000;0.0000;0.0000;0.0100;0.0000;0.0000;0.0000;0.0000
  3;0;10.0300;-7.1356;0.0;0.0100;0.0000;0.0000;0.0000;0.0100;0.0000;0.0000;0.0000;0.0000
  5;0;-5.9279;-7.9884;0.0;0.0100;0.0000;0.0000;0.0000;0.0100;0.0000;0.0000;0.0000;0.0000
  6;0;0.4598;-17.1013;0.0;0.0100;0.0000;0.0000;0.0000;0.0100;0.0000;0.0000;0.0000;0.0000
*/
TEST(FileParsingTests, parse_measurements)
{
  fs::path csv_path = measurements_path_dir / fs::path("measurements_1717000001.csv");
  auto measurements = parse_measurements(csv_path);
  EXPECT_EQ(measurements.size(), 5);
  EXPECT_FALSE(measurements.find(1) == measurements.end());
  EXPECT_FALSE(measurements.find(2) == measurements.end());
  EXPECT_FALSE(measurements.find(3) == measurements.end());
  EXPECT_FALSE(measurements.find(5) == measurements.end());
  EXPECT_FALSE(measurements.find(6) == measurements.end());
  EXPECT_NEAR(measurements[5].landmark.x, -5.9279, 1e-4);
  EXPECT_NEAR(measurements[1].landmark.y, 3.1875, 1e-4);
  for (const auto & [id, l] : measurements) {
    EXPECT_EQ(id, l.landmark.id);
    EXPECT_EQ(l.landmark.type, gpe_msgs::msg::Landmark2D::TYPE_XY);
    EXPECT_NEAR(l.covariance[0], 0.01, 1e-3);
    EXPECT_EQ(l.covariance[1], 0.0);
    EXPECT_EQ(l.covariance[3], 0.0);
    EXPECT_NEAR(l.covariance[4], 0.01, 1e-3);
    EXPECT_EQ(l.covariance[8], 0.0);
  }
}

/** Test SE2 measurements CSV parsing */
TEST(FileParsingTests, parse_measurements_se2)
{
  fs::path csv_path = measurements_path_dir_se2 / fs::path("measurements_1717000001.csv");
  auto measurements = parse_measurements(csv_path);
  EXPECT_EQ(measurements.size(), 5);
  EXPECT_FALSE(measurements.find(1) == measurements.end());
  EXPECT_FALSE(measurements.find(2) == measurements.end());
  EXPECT_FALSE(measurements.find(3) == measurements.end());
  EXPECT_FALSE(measurements.find(5) == measurements.end());
  EXPECT_FALSE(measurements.find(6) == measurements.end());
  EXPECT_NEAR(measurements[5].landmark.x, -5.9279, 1e-4);
  EXPECT_NEAR(measurements[1].landmark.y, 3.1875, 1e-4);
  for (const auto & [id, l] : measurements) {
    EXPECT_EQ(id, l.landmark.id);
    EXPECT_EQ(l.landmark.type, gpe_msgs::msg::Landmark2D::TYPE_SE2);
    EXPECT_NEAR(l.covariance[0], 0.01, 1e-3);
    EXPECT_EQ(l.covariance[1], 0.0);
    EXPECT_EQ(l.covariance[3], 0.0);
    EXPECT_NEAR(l.covariance[4], 0.01, 1e-3);
    EXPECT_NEAR(l.covariance[8], 0.04, 1e-3);
  }
  EXPECT_NEAR(measurements[3].landmark.theta, 0.75, 1e-4);
}

/** Test poses CSV parsing
  Contents of file poses.csv:
  1717000001;14.1;14.2;-1.1
  1717000002;14.0;6.9;1.2
  1717000003;10.2;-5.2;0.7
  1717000004;-9.5;4.7;0.9
  1717000005;-2.7;-0.6;2.5
  1717000006;-1.3;-0.7;-2.8
  1717000007;-0.8;-0.1;1.7
  1717000008;-9.4;-16.5;-0.6
*/
TEST(FileParsingTests, parse_poses)
{
  fs::path csv_path = measurements_path_dir / fs::path("poses.csv");
  auto poses_map = parse_poses(csv_path);
  EXPECT_EQ(poses_map.size(), 8);
  EXPECT_FALSE(poses_map.find(1717000001) == poses_map.end());
  EXPECT_FALSE(poses_map.find(1717000002) == poses_map.end());
  EXPECT_FALSE(poses_map.find(1717000003) == poses_map.end());
  EXPECT_FALSE(poses_map.find(1717000004) == poses_map.end());
  EXPECT_FALSE(poses_map.find(1717000005) == poses_map.end());
  EXPECT_FALSE(poses_map.find(1717000006) == poses_map.end());
  EXPECT_FALSE(poses_map.find(1717000007) == poses_map.end());
  EXPECT_FALSE(poses_map.find(1717000008) == poses_map.end());
  EXPECT_NEAR(poses_map[1717000006][0], -1.3, 1e-4);
  EXPECT_NEAR(poses_map[1717000001][1], 14.2, 1e-4);
  EXPECT_NEAR(poses_map[1717000003][2], 0.7, 1e-4);
}

/** Test write_poses() function */
TEST(FileParsingTests, write_poses)
{
  fs::path out_path = measurements_path_dir / fs::path("poses_out_test.csv");

  std::map<int64_t, Eigen::Vector3d> poses_map;
  poses_map[1717000001] = {14.1, 14.2, -1.1};
  poses_map[1717000002] = {14.0, 6.9, 1.2};
  poses_map[1717000003] = {10.2, -5.2, 0.7};

  write_poses(poses_map, out_path);

  ASSERT_TRUE(fs::exists(out_path));

  auto read_back = parse_poses(out_path);
  EXPECT_EQ(read_back.size(), poses_map.size());
  for (const auto & [timestamp, pose] : poses_map) {
    ASSERT_FALSE(read_back.find(timestamp) == read_back.end());
    EXPECT_NEAR(read_back[timestamp][0], pose[0], 1e-4);
    EXPECT_NEAR(read_back[timestamp][1], pose[1], 1e-4);
    EXPECT_NEAR(read_back[timestamp][2], pose[2], 1e-4);
  }

  fs::remove(out_path);
}

}  // namespace gpe


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
