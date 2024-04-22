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
: node_id_{0}, pose_id_{1000}
{
  initialize_optimizer();
}

SE2PoseEstimation::SE2PoseEstimation(const std::vector<Eigen::Vector2d> & landmarks)
: node_id_{0}, pose_id_{1000}
{
  initialize_optimizer();
  add_landmarks(landmarks);
}

SE2PoseEstimation::SE2PoseEstimation(
  const std::vector<Eigen::Vector2d> & landmarks,
  const std::vector<unsigned long> & ids)
: node_id_{0}, pose_id_{1000}
{
  initialize_optimizer();
  add_landmarks(landmarks, ids);
}

SE2PoseEstimation::~SE2PoseEstimation()
{
}

void SE2PoseEstimation::initialize_optimizer()
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

void SE2PoseEstimation::add_landmark(const Eigen::Vector2d & landmark)
{
  increase_node_id();
  landmarks_.push_back(landmark);
  add_landmark(landmark, node_id_);
}

void SE2PoseEstimation::add_landmark(
  const Eigen::Vector2d & landmark,
  const unsigned long id)
{
  // TODO: Check if ID exists. Change pose_id_ if there is conflict.
  landmarks_.push_back(landmark);
  // Add landmark vertex to optimizer
  g2o::VertexPointXY * landmark_vertex = new g2o::VertexPointXY;
  landmark_vertex->setId(id);
  landmark_ids_.push_back(id);
  landmark_vertex->setFixed(true);
  landmark_vertex->setEstimate(landmark);
  optimizer_.addVertex(landmark_vertex);
}

void SE2PoseEstimation::add_landmarks(const std::vector<Eigen::Vector2d> & landmarks)
{
  // TODO: Reserve space in vector of landmarks and IDs
  for (const auto & l : landmarks) {
    add_landmark(l);
  }
}

void SE2PoseEstimation::add_landmarks(
  const std::vector<Eigen::Vector2d> & landmarks,
  const std::vector<unsigned long> & ids
)
{
  // TODO
  (void) landmarks;
  (void) ids;
}

bool SE2PoseEstimation::check_id(const unsigned long id)
{
  return std::find(landmark_ids_.cbegin(), landmark_ids_.cend(), id) != landmark_ids_.cend();
}

void SE2PoseEstimation::increase_node_id()
{
  node_id_++;
  while (node_id_ == pose_id_ || check_id(node_id_)) {
    node_id_++;
  }
}


void SE2PoseEstimation::set_initial_pose(const g2o::SE2 & initial_pose)
{
  // TODO: Setting the initial pose should reset all poses (call reset_measurements)
  initial_pose_estimate_ = initial_pose;

  // Set vertex from robot pose (initial guess)
  // TODO: It should remove the previous vertex (if any) or reset its ID
  g2o::VertexSE2 * robot_pose_vertex = new g2o::VertexSE2;
  robot_pose_vertex->setId(pose_id_);  // Initial pose estimate always has ID zero
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
  landmark_observation->vertices()[0] = optimizer_.vertex(pose_id_);
  landmark_observation->vertices()[1] = optimizer_.vertex(landmark_id);

  landmark_observation->setMeasurement(measurement);
  landmark_observation->setInformation(inf_matrix);
  optimizer_.addEdge(landmark_observation);
}

void SE2PoseEstimation::reset_measurements()
{
  // TODO: Iterate over all measurements and remove them from the graph
}


g2o::SE2 SE2PoseEstimation::estimate()
{
  // Perform optimization
  optimizer_.initializeOptimization();
  optimizer_.optimize(100);

  // Compute error
  g2o::VertexSE2 * pose_vertex = dynamic_cast<g2o::VertexSE2 *>(optimizer_.vertex(pose_id_));
  return pose_vertex->estimate();

  // TODO: Update initial_pose for next iteration
}

} // namespace gpe
