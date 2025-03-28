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
#include <gpe_core/utils.hpp>

namespace gpe
{

using std::numbers::pi;
constexpr double float_tolerance = 1e-4;

TEST(GPEUtilsTests, test_deg_to_rad)
{
  ASSERT_EQ(deg_to_rad(0.0), 0.0);
  ASSERT_EQ(deg_to_rad(180.0), pi);
  ASSERT_EQ(deg_to_rad(360.0), 2 * pi);
}

TEST(GPEUtilsTests, test_rad_to_deg)
{
  ASSERT_EQ(rad_to_deg(0.0), 0.0);
  ASSERT_EQ(rad_to_deg(pi), 180.0);
  ASSERT_EQ(rad_to_deg(2 * pi), 360.0);
}

TEST(GPEUtilsTests, test_deg_to_rad_to_deg)
{
  double num_deg = 145.7;
  ASSERT_EQ(rad_to_deg(deg_to_rad(num_deg)), num_deg);

  double num_rad = 1.41;
  ASSERT_EQ(deg_to_rad(rad_to_deg(num_rad)), num_rad);
}

TEST(GPEUtilsTests, test_deg_to_rad_types)
{
  float f_num = 180.0;
  ASSERT_EQ(deg_to_rad(f_num), pi);

  int i_num = 180;
  ASSERT_EQ(deg_to_rad(i_num), pi);
}

TEST(GPEUtilsTests, test_norm_angle)
{
  // norm_angle normalizes in the interval [-pi, pi)
  double num = 1.6;
  ASSERT_NEAR(norm_angle(num), 1.6, float_tolerance);

  double neg_num = -1.6;
  ASSERT_NEAR(norm_angle(neg_num), -1.6, float_tolerance);

  double over_num = pi + 0.2;
  ASSERT_NEAR(norm_angle(over_num), -2.94159265, float_tolerance);

  double over_over_num = 2 * pi + 0.2;
  ASSERT_NEAR(norm_angle(over_over_num), 0.2, float_tolerance);

  ASSERT_NEAR(norm_angle(pi), -pi, float_tolerance);
  double neg_pi = -pi;
  ASSERT_NEAR(norm_angle(neg_pi), -pi, float_tolerance);
}

TEST(GPEUtilsTests, test_quaternion_from_yaw)
{
  double yaw_zero = 0.0;
  geometry_msgs::msg::Quaternion q1 = quaternion_msg_from_yaw(yaw_zero);
  // std::cout << "quat 1: " << q1.x << ", " << q1.y << ", " << q1.z << ", " << q1.w << std::endl;
  ASSERT_EQ(q1.x, 0.0);
  ASSERT_EQ(q1.y, 0.0);
  ASSERT_EQ(q1.z, 0.0);
  ASSERT_EQ(q1.w, 1.0);

  double yaw_pi = pi / 2;
  geometry_msgs::msg::Quaternion q2 = quaternion_msg_from_yaw(yaw_pi);
  // std::cout << "quat 2: " << q2.x << ", " << q2.y << ", " << q2.z << ", " << q2.w << std::endl;
  ASSERT_EQ(q2.x, 0.0);
  ASSERT_EQ(q2.y, 0.0);
  ASSERT_DOUBLE_EQ(q2.z, std::sqrt(2) / 2);
  ASSERT_DOUBLE_EQ(q2.w, std::sqrt(2) / 2);
}

TEST(GPEUtilsTests, test_yaw_from_quaternion)
{
  geometry_msgs::msg::Quaternion q1;
  q1.x = 0.0;
  q1.y = 0.0;
  q1.z = 0.0;
  q1.w = 1.0;
  ASSERT_DOUBLE_EQ(yaw_from_quaternion(q1), 0.0);

  geometry_msgs::msg::Quaternion q2;
  q2.x = 0.0;
  q2.y = 0.0;
  q2.z = std::sqrt(2) / 2;
  q2.w = std::sqrt(2) / 2;
  ASSERT_DOUBLE_EQ(yaw_from_quaternion(q2), pi / 2);
}

}  // namespace gpe


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
