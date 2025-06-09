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

namespace fs = std::filesystem;
namespace gpe
{
using Landmark2D = gpe_msgs::msg::Landmark2D;

TEST(LandmarkIOTests, file_parse_2d_xy_test)
{
  fs::path yaml_path = fs::path(TEST_DIRECTORY) / fs::path("test_landmarks_xy.yaml");
  ASSERT_NO_THROW(load_landmarks_2d(yaml_path));
  std::map<int, Landmark2D> landmarks = load_landmarks_2d(yaml_path);
  ASSERT_EQ(landmarks.size(), 3);
  for (const auto &[id, l] : landmarks) {
    ASSERT_EQ(l.id, id);
  }
  ASSERT_EQ(landmarks[1].x, -1.0);
  ASSERT_EQ(landmarks[1].y, 3.0);
}

TEST(LandmarkIOTests, file_parse_2d_se2_test)
{
  fs::path yaml_path = fs::path(TEST_DIRECTORY) / fs::path("test_landmarks_se2.yaml");
  ASSERT_NO_THROW(load_landmarks_2d(yaml_path));
  std::map<int, Landmark2D> landmarks = load_landmarks_2d(yaml_path);
  ASSERT_EQ(landmarks.size(), 3);
  for (const auto &[id, l] : landmarks) {
    ASSERT_EQ(l.id, id);
  }
  ASSERT_EQ(landmarks[2].x, -4.0);
  ASSERT_EQ(landmarks[1].y, 3.0);
  ASSERT_EQ(landmarks[8].theta, 1.5);
}


}  // namespace gpe


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
