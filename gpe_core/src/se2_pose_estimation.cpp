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

#include <unordered_set>

namespace gpe
{

SE2PoseEstimation::SE2PoseEstimation()
{
  initialize_optimizer();
}

SE2PoseEstimation::SE2PoseEstimation(const std::vector<Eigen::Vector2d> & landmarks)
{
  initialize_optimizer();
  add_landmarks(landmarks);
}

SE2PoseEstimation::SE2PoseEstimation(
  const std::vector<Eigen::Vector2d> & landmarks,
  const std::vector<unsigned long> & ids)
{
  initialize_optimizer();
  add_landmarks(landmarks, ids);
}


void SE2PoseEstimation::initialize_optimizer()
{
  // Setup optimizer algorithm and solver
  optimizer_.setVerbose(true);

  using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
  using SlamLinearSolver = g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType>;
  auto linearSolver = g2o::make_unique<SlamLinearSolver>();
  linearSolver->setBlockOrdering(false);

  auto solver = std::make_unique<g2o::OptimizationAlgorithmLevenberg>(
    std::make_unique<SlamBlockSolver>(std::move(linearSolver)));
  optimizer_.setAlgorithm(solver.release());
}

void SE2PoseEstimation::add_landmark(const Eigen::Vector2d & landmark)
{
  increase_node_id();
  landmarks_.push_back(landmark);
  add_landmark(landmark, node_id_);
}

bool SE2PoseEstimation::add_landmark(
  const Eigen::Vector2d & landmark,
  const unsigned long id)
{
  // Check if ID exists. Change pose_id_ if there is conflict.
  if (is_landmark_id(id)) {
    return false;
  }
  if (id == pose_id_) {
    update_pose_id();
  }

  landmarks_.push_back(landmark);
  // Add landmark vertex to optimizer
  g2o::VertexPointXY * landmark_vertex = new g2o::VertexPointXY;
  landmark_vertex->setId(id);
  landmark_ids_.push_back(id);
  landmark_vertex->setFixed(true);
  landmark_vertex->setEstimate(landmark);
  optimizer_.addVertex(landmark_vertex);
  return true;
}

void SE2PoseEstimation::add_landmarks(const std::vector<Eigen::Vector2d> & landmarks)
{
  // Reserve space in vector of landmarks and IDs
  landmarks_.reserve(landmarks_.size() + landmarks.size());
  landmark_ids_.reserve(landmark_ids_.size() + landmarks.size());
  for (const auto & l : landmarks) {
    add_landmark(l);
  }
}

bool SE2PoseEstimation::add_landmarks(
  const std::vector<Eigen::Vector2d> & landmarks,
  const std::vector<unsigned long> & ids
)
{
  assert(
    "Vector of landmarks and IDs size mismatch" &&
    landmarks.size() == ids.size()
  );

  // Check all IDs before adding any landmark to the graph
  // Initialize a set with the used landmarks
  std::unordered_set<unsigned long> unique_ids{landmark_ids_.begin(), landmark_ids_.end()};
  for (const auto & id : ids) {
    // Try to insert the new ID in the set, and return false if ID exists
    if (!unique_ids.insert(id).second) {
      return false;
    }
  }

  // At this point, all IDs are valid
  // Reserve space in vector of landmarks and IDs
  landmarks_.reserve(landmarks_.size() + landmarks.size());
  landmark_ids_.reserve(landmark_ids_.size() + landmarks.size());
  // Add landmarks with the specified IDs
  for (size_t i = 0; i < landmarks.size(); i++) {
    add_landmark(landmarks[i], ids[i]);
  }

  return true;
}


bool SE2PoseEstimation::is_landmark_id(const unsigned long id)
{
  return std::find(landmark_ids_.cbegin(), landmark_ids_.cend(), id) != landmark_ids_.cend();
}

void SE2PoseEstimation::increase_node_id()
{
  node_id_++;
  // If next ID is taken, keep incrementing until we find a free one
  while (node_id_ == pose_id_ || is_landmark_id(node_id_)) {
    node_id_++;
  }
}

void SE2PoseEstimation::update_pose_id()
{
  pose_id_++;
  // If next ID is taken, keep incrementing until we find a free one
  while (is_landmark_id(pose_id_)) {
    pose_id_++;
  }
}

void SE2PoseEstimation::set_initial_pose(const g2o::SE2 & initial_pose)
{
  // Set the current pose estimation
  pose_ = initial_pose;
  // Setting the initial pose removes all previous measurements
  reset_measurements();

  // Set vertex from robot pose (initial guess)
  if (auto v = optimizer_.vertex(pose_id_)) {
    optimizer_.removeVertex(v);
  }
  g2o::VertexSE2 * robot_pose_vertex = new g2o::VertexSE2;
  robot_pose_vertex->setId(pose_id_);
  robot_pose_vertex->setEstimate(initial_pose);
  optimizer_.addVertex(robot_pose_vertex);
}


void SE2PoseEstimation::add_measurement(
  const Eigen::Vector2d & measurement,
  const Eigen::Matrix2d & inf_matrix)
{
  // TODO: Association required
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
  auto edges = optimizer_.edges();
  for (const auto & edge : edges) {
    optimizer_.removeEdge(edge);
  }
}


g2o::SE2 SE2PoseEstimation::estimate()
{
  // Perform optimization
  optimizer_.initializeOptimization();
  optimizer_.optimize(100);

  // Compute error
  g2o::VertexSE2 * pose_vertex = dynamic_cast<g2o::VertexSE2 *>(optimizer_.vertex(pose_id_));

  // Update initial_pose for next iteration
  pose_ = pose_vertex->estimate();

  // Remove measurements, as they are not valid for the next iteration (pose is different)
  reset_measurements();

  return pose_;
}

} // namespace gpe
