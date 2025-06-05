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

namespace gpe
{

/** Load a set of landmarks from a YAML file.
    Returns a map where the key is the landmark ID and the value is the landmark.
*/
std::map<int, gpe_msgs::msg::Landmark2D> load_landmarks(const std::string & filename);

}  // namespace gpe

#endif  // GPE_LANDDMARK_SERVER__LANDMARK_IO_HPP_
