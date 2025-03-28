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

#include <random>

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/utils.h>

#include <geometry_msgs/msg/quaternion.hpp>

namespace gpe
{

namespace
{
// Anonymous namespace to avoid exporting this
using std::numbers::pi;
}

/** Creates and returns a quaternion msg from a given yaw angle */
inline geometry_msgs::msg::Quaternion quaternion_msg_from_yaw(double yaw)
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
inline double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  return tf2::getYaw(tf_q);
}

/** Converts degrees to radians in compile time */
constexpr double deg_to_rad(const double deg)
{
  return std::numbers::pi * deg / 180.0;
}

/** Converts radians to degrees in compile time */
constexpr double rad_to_deg(const double rad)
{
  return 180.0 * rad / pi;
}

/** Angle normalization to [0, 2*pi) range (in radians) */
constexpr double norm_angle_2pi(const double angle)
{
  double norm_angle = std::fmod(angle, 2 * pi);
  return norm_angle < 0 ? norm_angle + deg_to_rad(360.0) : norm_angle;
}

/** Angle normalization to [-pi, pi) range (in radians) */
constexpr double norm_angle(const double angle)
{
  double norm_angle = std::fmod(angle + pi, 2 * pi);
  return norm_angle < 0 ? norm_angle + pi : norm_angle - pi;
}

/* Random utils */
static std::mt19937 get_random_generator()
{
  static std::random_device rd;
  static std::mt19937 gen(rd());
  return gen;
}

inline double uniform_rand(double low, double high)
{
  static std::mt19937 gen = get_random_generator();
  std::uniform_real_distribution<> dis(low, high);
  return dis(gen);
}

inline double gaussian(double sigma)
{
  static std::mt19937 gen = get_random_generator();
  std::normal_distribution<> dis(0.0, sigma);
  return dis(gen);
}


}  // namespace gpe

#endif  // GPE_CORE__UTILS_HPP_
