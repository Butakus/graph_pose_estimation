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

#ifndef GPE_CORE__UTILS_HPP_
#define GPE_CORE__UTILS_HPP_

#include <tf2/LinearMath/Quaternion.h>
// Remove "-Wpedantic with tf2/utils.h to avoid warnings about extra ';'"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <tf2/utils.h>
#pragma GCC diagnostic pop

#include <cmath>
#include <geometry_msgs/msg/quaternion.hpp>

namespace gpe
{

/** Creates and returns a quaternion msg from a given yaw angle */
geometry_msgs::msg::Quaternion quaternion_msg_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  tf2::Quaternion tf_q;

  tf_q.setRPY(0.0, 0.0, yaw);

  q.x = tf_q.x();
  q.y = tf_q.y();
  q.z = tf_q.z();
  q.w = tf_q.w();

  return q;
}

/** Extracts yaw angle from quaternion msg */
double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  return tf2::getYaw(tf_q);
}

/** Converts degrees to radians in compile time */
constexpr double deg_to_rad(const double deg)
{
  return M_PI * deg / 180.0;
}

/** Converts radians to degrees in compile time */
constexpr double rad_to_deg(const double rad)
{
  return 180.0 * rad / M_PI;
}

/** Angle normalization to [0-2PI] range (in radians) */
constexpr double norm_angle(const double angle)
{
  double normalized_angle = std::fmod(angle, 2 * M_PI);
  return normalized_angle < 0 ? normalized_angle + deg_to_rad(360.0) : normalized_angle;
}


}  // namespace gpe

#endif  // GPE_CORE__UTILS_HPP_
