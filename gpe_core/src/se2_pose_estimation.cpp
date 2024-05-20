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

#include <gpe_core/se2_pose_estimation.hpp>
#include <gpe_core/hungarian.hpp>

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
  optimizer_.setVerbose(false);

  using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
  using SlamLinearSolver = g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType>;
  auto linearSolver = std::make_unique<SlamLinearSolver>();
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

bool SE2PoseEstimation::add_landmark(const gpe_msgs::msg::Landmark & landmark_msg)
{
  Eigen::Vector2d landmark {landmark_msg.x, landmark_msg.y};
  return add_landmark(landmark, landmark_msg.id);
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

bool SE2PoseEstimation::add_landmarks(const gpe_msgs::msg::LandmarkArray & landmarks_msg)
{
  std::vector<Eigen::Vector2d> landmarks;
  std::vector<unsigned long> ids;
  for (const auto & landmark_msg : landmarks_msg.landmarks) {
    landmarks.emplace_back(landmark_msg.x, landmark_msg.y);
    ids.push_back(landmark_msg.id);
  }
  return add_landmarks(landmarks, ids);
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
  g2o::EdgeSE2PointXY * landmark_observation = new g2o::EdgeSE2PointXY;

  // The second vertex (landmark ID) will be set in the association step.
  landmark_observation->vertices()[0] = optimizer_.vertex(pose_id_);

  landmark_observation->setMeasurement(measurement);
  landmark_observation->setInformation(inf_matrix);
  detached_measurements_.push_back(landmark_observation);
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
    // This removes the edge from the graph and its connections to any vertex
    optimizer_.removeEdge(edge);
  }
  // Measurements without ID are not stored in the optimizer.
  // We must release the memory and clear the vector
  for (auto & edge : detached_measurements_) {
    delete edge;
  }
  detached_measurements_.clear();
}

void SE2PoseEstimation::associate_detached_measurements()
{
  // Check which landmarks already have a measurement attached and skip them
  std::vector<unsigned long> free_ids;
  std::vector<Eigen::Vector2d> free_landmarks;
  free_ids.reserve(landmark_ids_.size());
  for (const auto & id : landmark_ids_) {
    // Only use the landmarks that are not connected
    auto landmark = dynamic_cast<g2o::VertexPointXY *>(optimizer_.vertex(id));
    if (landmark->edges().size() == 0) {
      free_ids.push_back(id);
      free_landmarks.push_back(landmark->estimate());
    }
  }

  // Initialize cost matrix
  std::vector<std::vector<double>> cost_matrix(
    detached_measurements_.size(),
    std::vector<double>(free_ids.size())
  );

  // Compute measurement errors and build the cost matrix
  for (size_t i = 0; i < detached_measurements_.size(); i++) {
    for (size_t j = 0; j < free_ids.size(); j++) {
      detached_measurements_[i]->vertices()[1] = optimizer_.vertex(free_ids[j]);
      // Use G2O error computation (_error is a Vector2d for this edge type)
      detached_measurements_[i]->computeError();
      // Use Chi square error: _error.dot(information() * _error)
      cost_matrix[i][j] = detached_measurements_[i]->chi2();
    }
  }

  gpe::HungarianAssignment hungarian(cost_matrix);
  std::vector<size_t> assignment;
  hungarian.assign(assignment);

  // TODO: Maybe filter out outliers (false positives).
  // Outliers are measurements associated to a landmark that is far.

  // Go over the detached edges, set their assigned ID and add them to the graph
  for (size_t i = 0; i < assignment.size(); i++) {
    // Attach the measurement to the assigned landmark
    auto assigned_landmark = optimizer_.vertex(free_ids[assignment[i]]);
    detached_measurements_[i]->vertices()[1] = assigned_landmark;
    // Recompute error to avoid a messed up state
    detached_measurements_[i]->computeError();
    // Add the edge to the optimizer, transfering ownership
    optimizer_.addEdge(detached_measurements_[i]);
  }
  // We don't need these pointers anymore. Ownership is transfered to the Optimizer.
  detached_measurements_.clear();
}

g2o::SE2 SE2PoseEstimation::estimate()
{
  // Run association algrithm to attach the measurements without landmark the graph
  if (detached_measurements_.size() > 0) {
    associate_detached_measurements();
  }
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
