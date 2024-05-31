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
#include <random>

#include <gtest/gtest.h>
#include <gpe_landmark_server/landmark_io.hpp>
#include <gpe_offline_estimation/file_io.hpp>

namespace fs = std::filesystem;
namespace gpe
{

// Paths
const fs::path landmarks_path = TEST_DIRECTORY / fs::path("data/test_landmarks.yaml");
const fs::path measurements_path_dir = TEST_DIRECTORY / fs::path("data/measurements");


/** Test find_measurement_files function from <gpe_offline_estimation/file_io.hpp> */
TEST(FileParsingTests, find_measurement_files)
{
  // Check if data files exist and remove them
  ASSERT_TRUE(fs::is_directory(measurements_path_dir));
  auto measurement_paths = find_measurement_files(measurements_path_dir);
  ASSERT_TRUE(measurement_paths.size() == 8);
  for (const auto & p : measurement_paths) {
    EXPECT_TRUE(p.extension() == ".csv");
    EXPECT_EQ(p.filename().string().rfind("measurements_", 0), 0);
  }
}

/** Test landmark loading from YAML file (using gpe_landmark_server utils) */
TEST(FileParsingTests, load_landmarks)
{
  // Load map of landmarks from YAML file
  auto landmarks_map = gpe::load_landmarks(landmarks_path);
  ASSERT_EQ(landmarks_map.size(), 10);
  for (const auto &[id, l] : landmarks_map) {
    EXPECT_EQ(l.id, id);
  }
  EXPECT_EQ(landmarks_map[1].x, 16.3);
  EXPECT_EQ(landmarks_map[1].y, 16.9);
  EXPECT_EQ(landmarks_map[9].x, -8.2);
  EXPECT_EQ(landmarks_map[9].y, 6.3);
}


}  // namespace gpe


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
