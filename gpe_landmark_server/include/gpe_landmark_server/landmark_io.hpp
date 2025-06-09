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

#ifndef GPE_LANDDMARK_SERVER__LANDMARK_IO_HPP_
#define GPE_LANDDMARK_SERVER__LANDMARK_IO_HPP_

#include <map>
#include <gpe_msgs/msg/landmark2_d.hpp>
#include <unordered_map>

namespace gpe
{

/**
 * Load a set of landmarks from a YAML file.
 *
 * @param filename the pat hto the YAML file with the landmark data
 * @return A map where the key is the landmark ID and the value is the Landmark2D object.
 */
std::map<int, gpe_msgs::msg::Landmark2D> load_landmarks_2d(const std::string & filename);
// std::map<int, gpe_msgs::msg::Landmark2D> load_landmarks_3d(const std::string & filename);

namespace landmark_io
{
  static const std::unordered_map<std::string, size_t> landmark_type_size
  {
    {"PointXY", 2},    // x, y
    {"Pose2D", 3},     // x, y, theta
    // Not supported yet
    // {"PointXYZ", 3},   // x, y, z
    // {"Pose3D", 6},     // x, y, z, roll, pitch, yaw
    // {"Pose3DQuat", 7}  // x, y, z, qx, qy, qz, qw
  };

  static const std::unordered_map<std::string, unsigned char> landmark_type_conversions
  {
    {"PointXY", gpe_msgs::msg::Landmark2D::TYPE_XY},
    {"Pose2D", gpe_msgs::msg::Landmark2D::TYPE_SE2},
    // Not supported yet
    // {"PointXYZ", 3},
    // {"Pose3D", 6},
    // {"Pose3DQuat", 7}
  };
}  // namespace landmark_io

}  // namespace gpe

#endif  // GPE_LANDDMARK_SERVER__LANDMARK_IO_HPP_
