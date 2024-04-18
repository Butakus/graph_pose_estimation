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

#include "gpe_core/se2_pose_estimation.hpp"
#include <gpe_core/utils.hpp>

namespace gpe
{

SE2PoseEstimation::SE2PoseEstimation()
: node_id_(1)
{
  // Setup optimizer algorithm and solver
  optimizer_.setVerbose(false);

  using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
  using SlamLinearSolver = g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType>;
  auto linearSolver = g2o::make_unique<SlamLinearSolver>();
  linearSolver->setBlockOrdering(false);

  // auto solver = std::make_unique<g2o::OptimizationAlgorithmLevenberg>(
  //   g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));
  // optimizer_.setAlgorithm(solver.get());

  g2o::OptimizationAlgorithmLevenberg * solver =
    new g2o::OptimizationAlgorithmLevenberg(
    g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));
  optimizer_.setAlgorithm(solver);

}

SE2PoseEstimation::~SE2PoseEstimation()
{
}

void SE2PoseEstimation::add_landmark(const Eigen::Vector2d & landmark)
{
  landmarks_.push_back(landmark);
  add_landmark_to_graph(landmark);
}

void SE2PoseEstimation::add_landmarks(const std::vector<Eigen::Vector2d> & landmarks)
{
  // TODO: Reserve space in vector of landmarks and IDs
  for (const auto & l : landmarks) {
    add_landmark(l);
  }
}

void SE2PoseEstimation::add_landmark_to_graph(const Eigen::Vector2d & landmark)
{
  // Add landmark vertex to optimizer
  g2o::VertexPointXY * landmark_vertex = new g2o::VertexPointXY;
  landmark_vertex->setId(node_id_);
  landmark_ids_.push_back(node_id_);
  node_id_++;
  landmark_vertex->setFixed(true);
  landmark_vertex->setEstimate(landmark);
  optimizer_.addVertex(landmark_vertex);
}

/** Removes all elements from the graph except the landmark */
void SE2PoseEstimation::reset_graph()
{
  // TODO: Remove everything, reset ID counter, and add the landmarks again to the graph
}

void SE2PoseEstimation::set_initial_pose(const g2o::SE2 & initial_pose)
{
  // TODO: Setting the initial pose should reset all poses (call reset_graph)
  initial_pose_estimate_ = initial_pose;

  // Set vertex from robot pose (initial guess)
  g2o::VertexSE2 * robot_pose_vertex = new g2o::VertexSE2;
  robot_pose_vertex->setId(0);  // Initial pose estimate always has ID zero
  robot_pose_vertex->setEstimate(initial_pose);
  optimizer_.addVertex(robot_pose_vertex);
}

void SE2PoseEstimation::add_measurement(
  const Eigen::Vector2d & measurement,
  const Eigen::Matrix2d & inf_matrix)
{
  // TODO
  (void) measurement;
  (void) inf_matrix;
}

void SE2PoseEstimation::add_measurement(
  const Eigen::Vector2d & measurement,
  const Eigen::Matrix2d & inf_matrix,
  const unsigned long landmark_id)
{
  g2o::EdgeSE2PointXY * landmark_observation = new g2o::EdgeSE2PointXY;
  landmark_observation->vertices()[0] = optimizer_.vertex(0);
  landmark_observation->vertices()[1] = optimizer_.vertex(landmark_id);  // TODO: Fix

  landmark_observation->setMeasurement(measurement);
  landmark_observation->setInformation(inf_matrix);
  optimizer_.addEdge(landmark_observation);
}


g2o::SE2 SE2PoseEstimation::estimate()
{
  // Perform optimization
  optimizer_.initializeOptimization();
  optimizer_.optimize(100);

  // Compute error
  g2o::VertexSE2 * pose_vertex = dynamic_cast<g2o::VertexSE2 *>(optimizer_.vertex(0));
  return pose_vertex->estimate();
}

} // namespace gpe
