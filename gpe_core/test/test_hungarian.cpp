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
#include <gpe_core/hungarian.hpp>

TEST(HungarianTests, simple_association_test)
{
  std::vector<std::vector<double>> distances = {
    {10, 19, 8, 15, 0},
    {10, 18, 7, 17, 0},
    {13, 16, 9, 14, 0},
    {12, 19, 8, 18, 0}
  };

  gpe::HungarianAssignment hungarian(distances);
  std::vector<size_t> assignment;

  double total_cost = hungarian.assign(assignment);

  ASSERT_EQ(total_cost, 31.0);
  ASSERT_EQ(assignment.size(), 4);
  ASSERT_EQ(assignment[0], 0);
  ASSERT_EQ(assignment[1], 2);
  ASSERT_EQ(assignment[2], 3);
  ASSERT_EQ(assignment[3], 4);
}

TEST(HungarianTests, asymetric_association_test)
{
  std::vector<std::vector<double>> distances = {
    {9999.0, 1.0, 0.5},
    {9999.0, 6.0, 4.5}
  };

  gpe::HungarianAssignment hungarian(distances);
  std::vector<size_t> assignment;

  double total_cost = hungarian.assign(assignment);

  ASSERT_EQ(total_cost, 5.5);
  ASSERT_EQ(assignment.size(), 2);
  ASSERT_EQ(assignment[0], 1);
  ASSERT_EQ(assignment[1], 2);

  // std::cout << std::endl << "Distance matrix: " << std::endl;
  // for (const auto & v : distances) {
  //   for (const auto & d : v) {
  //     std::cout << d << "\t";
  //   }
  //   std::cout << std::endl;
  // }

  // std::cout << std::endl << "Assignment: " << std::endl;
  // for (size_t i = 0; i < assignment.size(); i++) {
  //   std::cout << i << " --> " << assignment[i] << std::endl;
  // }
  // std::cout << std::endl << "Total cost: " << total_cost << std::endl;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
