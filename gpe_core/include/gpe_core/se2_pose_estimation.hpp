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

#ifndef GPE_CORE__SE2_POSE_ESTIMATION_HPP_
#define GPE_CORE__SE2_POSE_ESTIMATION_HPP_

#include <Eigen/StdVector>
#include <Eigen/Dense>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam2d/se2.h>
#include <g2o/types/slam2d/vertex_point_xy.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2_pointxy.h>

#include <iostream>
#include <vector>
#include <cmath>

namespace gpe
{

class SE2PoseEstimation
{
public:
  SE2PoseEstimation();

  virtual ~SE2PoseEstimation();

  void add_landmark(const Eigen::Vector2d & landmark);
  void add_landmarks(const std::vector<Eigen::Vector2d> & landmarks);
  inline std::vector<Eigen::Vector2d> get_landmarks() const {return landmarks_;}

  void set_initial_pose(const g2o::SE2 & initial_pose);

  void add_measurement(
    const Eigen::Vector2d & measurement,
    const Eigen::Matrix2d & inf_matrix
  );

  void add_measurement(
    const Eigen::Vector2d & measurement,
    const Eigen::Matrix2d & inf_matrix,
    const unsigned long landmark_id
  );

  void reset_graph();

  g2o::SE2 estimate();

  g2o::SparseOptimizer & get_optimizer() {return optimizer_;}

private:
  void add_landmark_to_graph(const Eigen::Vector2d & landmark);
  // State (map and poses)
  std::vector<Eigen::Vector2d> landmarks_;
  g2o::SE2 initial_pose_estimate_;

  // G2O graph IDs and lookup tables
  unsigned long node_id_;  // Node IDs start at 1. ID 0 is reserved for initial estimate.
  std::vector<unsigned long> landmark_ids_;
  std::vector<unsigned long> pose_ids_;
  // Optimization objects
  g2o::SparseOptimizer optimizer_;
};

}  // namespace gpe

#endif  // GPE_CORE__SE2_POSE_ESTIMATION_HPP_
