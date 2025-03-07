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
//
// Note: This is a clean C++ adaptation of the Hungarian algorithm implemented by Cong Ma (BSD License):
//   https://github.com/mcximing/hungarian-algorithm-cpp.
// At the same time, that version is an adaptation of this
// original MATLAB code written by Markus Buehren (BSD License):
//   http://www.mathworks.com/matlabcentral/fileexchange/6543-functions-for-the-rectangular-assignment-problem

#include <gpe_core/hungarian.hpp>
#include <algorithm>

namespace gpe
{

// TODO: Make it usable multiple times

HungarianAssignment::HungarianAssignment(const Matrix<double> & cost)
{
  assert(cost.size() > 0);
  assert(cost[0].size() > 0);
  rows_ = cost.size();
  cols_ = cost[0].size();
  min_dim_ = std::min(rows_, cols_);
  cost_ = cost;
  distance_ = cost;
  star_ = Matrix<bool>(rows_, std::vector<bool>(cols_, false));
  prime_ = Matrix<bool>(rows_, std::vector<bool>(cols_, false));
  covered_rows_ = std::vector<bool>(rows_, false);
  covered_cols_ = std::vector<bool>(cols_, false);
}

double HungarianAssignment::assign(std::vector<size_t> & assignment)
{
  // First, create initial zeros
  init_zeros();

  // If solution is not valid yet, continue with the algorithm
  if (!check_solution()) {
    // Find zeros will repeat the algorithm steps until a valid solution is found
    find_zeros();
  }

  // Valid solution
  assignment.clear();
  assignment.resize(rows_);
  for (size_t i = 0; i < rows_; i++) {
    for (size_t j = 0; j < cols_; j++) {
      if (star_[i][j]) {
        assignment[i] = j;
        break;
      }
    }
  }
  double total_cost = 0.0;
  for (size_t i = 0; i < rows_; i++) {
    total_cost += distance_[i][assignment[i]];
  }
  return total_cost;
}

bool HungarianAssignment::check_solution()
{
  // count covered columns
  size_t covered = std::count(covered_cols_.begin(), covered_cols_.end(), true);
  return covered == min_dim_;
}

bool is_zero(double value)
{
  return std::abs(value) <= std::numeric_limits<double>::epsilon() * std::abs(value) ||
         std::abs(value) < std::numeric_limits<double>::min();
}

void HungarianAssignment::init_zeros()
{
  if (rows_ <= cols_) {
    for (size_t i = 0; i < rows_; i++) {
      // Find smallest element in the row
      double min_value = *std::min_element(cost_[i].begin(), cost_[i].end());
      // Substract smallest to all elements in the row
      for (size_t j = 0; j < cols_; j++) {
        cost_[i][j] -= min_value;
      }
    }
    for (size_t i = 0; i < rows_; i++) {
      for (size_t j = 0; j < cols_; j++) {
        // Find and mark zeros
        if (is_zero(cost_[i][j]) && !covered_cols_[j]) {
          star_[i][j] = true;
          covered_cols_[j] = true;
          break;
        }
      }
    }
  } else {
    for (size_t j = 0; j < cols_; j++) {
      // Find smallest element in the col
      double min_value = cost_[0][j];
      for (size_t i = 1; i < rows_; i++) {
        min_value = std::min(min_value, cost_[i][j]);
      }
      // Substract smallest to all elements in the col
      for (size_t i = 0; i < rows_; i++) {
        cost_[i][j] -= min_value;
      }
    }
    for (size_t j = 0; j < cols_; j++) {
      for (size_t i = 0; i < rows_; i++) {
        // Find and mark zeros
        if (is_zero(cost_[i][j]) && !covered_rows_[i]) {
          star_[i][j] = true;
          covered_rows_[i] = true;
          covered_cols_[j] = true;
          break;
        }
      }
    }
    // Reset covered_rows_
    std::fill(covered_rows_.begin(), covered_rows_.end(), false);
  }
}

void HungarianAssignment::find_zeros()
{
  bool zeros_found = true;

  while (zeros_found) {
    zeros_found = false;
    for (size_t j = 0; j < cols_; j++) {
      if (!covered_cols_[j]) {
        for (size_t i = 0; i < rows_; i++) {
          if ((!covered_rows_[i]) && is_zero(cost_[i][j])) {
            // Prime zero
            prime_[i][j] = true;

            // Find starred zero in current row
            int star_col = -1;
            for (size_t jj = 0; jj < cols_; jj++) {
              if (star_[i][jj]) {
                star_col = jj;
                break;
              }
            }
            // If found, mark row, unmark col and finish col processing
            if (star_col != -1) {
              covered_rows_[i] = true;
              covered_cols_[star_col] = false;
              zeros_found = true;
              break;
            } else {
              // No starred zeros found. Need to star zeros
              star_zeros(i, j);
              return;
            }
          }
        }
      }
    }
  }
  // Make new zeros
  make_zeros();
}

void HungarianAssignment::make_zeros()
{
  // Find smallest uncovered element
  double min_value = std::numeric_limits<double>::max();
  for (size_t i = 0; i < rows_; i++) {
    if (!covered_rows_[i]) {
      for (size_t j = 0; j < cols_; j++) {
        if (!covered_cols_[j]) {
          min_value = std::min(min_value, cost_[i][j]);
        }
      }
    }
  }

  // Add min_value to each covered row
  for (size_t i = 0; i < rows_; i++) {
    if (covered_rows_[i]) {
      for (size_t j = 0; j < cols_; j++) {
        cost_[i][j] += min_value;
      }
    }
  }

  // Subtract min_value from each uncovered column
  for (size_t j = 0; j < cols_; j++) {
    if (!covered_cols_[j]) {
      for (size_t i = 0; i < rows_; i++) {
        cost_[i][j] -= min_value;
      }
    }
  }

  // Repeat cycle
  find_zeros();
}

void HungarianAssignment::star_zeros(size_t row, size_t col)
{
  // Generate a temporary copy of the star matrix
  Matrix<bool> new_star = star_;

  // Star current zero
  new_star[row][col] = true;

  // Find a starred zero in current column
  int star_col = col;
  int star_row = -1;
  for (size_t i = 0; i < rows_; i++) {
    if (star_[i][star_col]) {
      star_row = i;
      break;
    }
  }

  while (star_row != -1) {
    // Unstar the starred zero
    new_star[star_row][star_col] = false;

    // Find primed zero in current row
    int prime_col = -1;
    for (size_t j = 0; j < cols_; j++) {
      if (prime_[star_row][j]) {
        prime_col = j;
        break;
      }
    }
    // There must be a primed zero in the row
    assert(prime_col != -1);
    // Star the primed zero
    new_star[star_row][prime_col] = true;

    // Find again a starred zero in current column
    star_col = prime_col;
    star_row = -1;
    for (size_t i = 0; i < rows_; i++) {
      if (star_[i][star_col]) {
        star_row = i;
        break;
      }
    }
  }

  // Update star matrix, clear primes and unciver rows
  for (size_t i = 0; i < rows_; i++) {
    // Clear primes
    std::fill(prime_[i].begin(), prime_[i].end(), false);
    // Update stars
    star_[i] = std::move(new_star[i]);
    // Uncover row
    covered_rows_[i] = false;
  }

  // Cover columns and check if solution is valid
  cover_cols();
  if (!check_solution()) {
    // If solution is not valid yet, repeat the cycle
    find_zeros();
  }
}

/* Cover every column containing a starred zero */
void HungarianAssignment::cover_cols()
{
  for (size_t j = 0; j < cols_; j++) {
    for (size_t i = 0; i < rows_; i++) {
      if (star_[i][j]) {
        covered_cols_[j] = true;
        break;
      }
    }
  }
}

}  // namespace gpe
