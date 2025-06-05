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

TEST(LandmarkIOTests, file_parse_test)
{
  using Landmark = gpe_msgs::msg::Landmark2D;
  fs::path yaml_path = fs::path(TEST_DIRECTORY) / fs::path("test_landmarks.yaml");
  std::map<int, Landmark> landmarks = load_landmarks(yaml_path);
  ASSERT_NO_THROW(landmarks = load_landmarks(yaml_path));
  ASSERT_EQ(landmarks.size(), 3);
  for (const auto &[id, l] : landmarks) {
    ASSERT_EQ(l.id, id);
  }
  ASSERT_EQ(landmarks[1].x, -1.0);
  ASSERT_EQ(landmarks[1].y, 3.0);
}


}  // namespace gpe


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
