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


namespace gpe
{

SE2PoseEstimation::SE2PoseEstimation()
{
  initialize_optimizer();
}

SE2PoseEstimation::SE2PoseEstimation(const LandmarkArray & landmarks)
{
  initialize_optimizer();
  add_landmarks(landmarks);
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

  // Create a vertex in the graph for the robot pose (with a zero/unknown estimate)
  set_initial_pose(g2o::SE2());
}

bool SE2PoseEstimation::add_landmark(const Landmark & landmark_msg)
{
  // If the landmark does not have an ID, assign an automatic ID
  int landmark_id = landmark_msg.id;
  if (landmark_id == -1) {
    increase_node_id();
    landmark_id = node_id_;
  }

  // Change pose_id_ if there is conflict.
  if (landmark_id == static_cast<int>(pose_id_)) {
    update_pose_id();
  }

  Eigen::Vector2d landmark {landmark_msg.x, landmark_msg.y};

  // Add landmark vertex to optimizer
  // TODO: Change vertex type according to landmark type
  g2o::VertexPointXY * landmark_vertex = new g2o::VertexPointXY();
  landmark_vertex->setId(landmark_id);
  landmark_vertex->setFixed(true);
  landmark_vertex->setEstimate(landmark);
  bool result_ok = optimizer_.addVertex(landmark_vertex);
  if (result_ok) {
    // If the vertex was successfully added, save the ID
    landmark_ids_.insert(landmark_id);
  } else {
    delete landmark_vertex;
  }
  return result_ok;
}

bool SE2PoseEstimation::add_landmarks(const LandmarkArray & landmarks_msg)
{
  // Check all IDs before adding any landmark to the graph
  std::set<unsigned int> unique_ids;
  size_t positive_count = 0;
  for (const auto & landmark : landmarks_msg.landmarks) {
    // Add positive IDs to a different set to check for duplicates
    if (landmark.id >= 0) {
      positive_count++;
      unique_ids.insert(static_cast<unsigned int>(landmark.id));
    }
    // Check if the new ID is already in the set, and return false if ID exists
    if (landmark_ids_.contains(landmark.id)) {
      return false;
    }
  }
  // Then, check if the list of IDs has duplicates
  if (unique_ids.size() != positive_count) {
    return false;
  }

  // At this point, all IDs are valid or unknown (-1)
  for (const auto & landmark : landmarks_msg.landmarks) {
    add_landmark(landmark);
  }

  return true;
}

gpe_msgs::msg::Landmark2DArray SE2PoseEstimation::get_landmarks() const
{
  LandmarkArray landmarks;
  landmarks.landmarks.reserve(optimizer_.vertices().size());
  for (const auto & [id, vertex] : optimizer_.vertices()) {
    // TODO: Check for different types of landmark
    auto vertex_xy = dynamic_cast<g2o::VertexPointXY *>(vertex);
    // Only extract fixed vertices that have an ID different from the robot pose
    if (id != static_cast<int>(pose_id_) && vertex_xy != nullptr && vertex_xy->fixed()) {
      Eigen::Vector2d position = vertex_xy->estimate();
      Landmark l;
      l.id = id;
      l.type = Landmark::TYPE_XY;
      l.x = position.x();
      l.y = position.y();
      l.theta = 0.0;
      landmarks.landmarks.push_back(l);
    }
  }
  return landmarks;
}

void SE2PoseEstimation::increase_node_id()
{
  node_id_++;
  // If next ID is taken, keep incrementing until we find a free one
  while (node_id_ == pose_id_ || landmark_ids_.contains(node_id_)) {
    node_id_++;
  }
}

void SE2PoseEstimation::update_pose_id()
{
  // Get the graph vertex with the old ID
  auto robot_pose_vertex = dynamic_cast<g2o::VertexSE2 *>(optimizer_.vertex(pose_id_));
  // Update the current ID, incrementing until a free one is found
  pose_id_++;
  // If next ID is taken, keep incrementing until we find a free one
  while (landmark_ids_.contains(pose_id_)) {
    pose_id_++;
  }
  // Update the ID for the robot pose Vertex inside the graph
  if (robot_pose_vertex != nullptr) {
    optimizer_.changeId(robot_pose_vertex, pose_id_);
  }
}

void SE2PoseEstimation::set_initial_pose(const g2o::SE2 & initial_pose)
{
  // Set the current pose estimation
  pose_ = initial_pose;
  // Setting the initial pose removes all previous measurements
  reset_measurements();

  // Update vertex estimate from robot pose (initial guess)
  auto robot_pose_vertex = dynamic_cast<g2o::VertexSE2 *>(optimizer_.vertex(pose_id_));
  if (robot_pose_vertex == nullptr) {
    // If vertex was not created yet, create a new one
    robot_pose_vertex = new g2o::VertexSE2();
    robot_pose_vertex->setId(pose_id_);
    optimizer_.addVertex(robot_pose_vertex);
  }
  robot_pose_vertex->setEstimate(initial_pose);
}


void SE2PoseEstimation::add_measurement(const MeasurementXY & measurement)
{
  g2o::EdgeSE2PointXY * landmark_observation = new g2o::EdgeSE2PointXY();

  // The second vertex (landmark ID) will be set in the association step.
  landmark_observation->vertices()[0] = optimizer_.vertex(pose_id_);

  landmark_observation->setMeasurement(measurement.data);
  landmark_observation->setInformation(measurement.inf_matrix);
  detached_measurements_.push_back(landmark_observation);
}

bool SE2PoseEstimation::add_measurement(
  const MeasurementXY & measurement,
  const unsigned int landmark_id
)
{
  auto landmark_vertex = optimizer_.vertex(landmark_id);
  if (landmark_vertex == nullptr) {
    return false;
  }
  MeasurementXY::EdgeType * landmark_observation = new MeasurementXY::EdgeType();
  landmark_observation->vertices()[0] = optimizer_.vertex(pose_id_);
  landmark_observation->vertices()[1] = landmark_vertex;

  landmark_observation->setMeasurement(measurement.data);
  landmark_observation->setInformation(measurement.inf_matrix);
  return optimizer_.addEdge(landmark_observation);
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
  std::vector<unsigned int> free_ids;
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
