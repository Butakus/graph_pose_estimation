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

#include <gpe_msgs/msg/landmark_array.hpp>

#include <vector>
#include <cmath>

namespace gpe
{

class SE2PoseEstimation
{
public:
  /** Default constructor */
  SE2PoseEstimation();
  /** Constructor with a given set of landmarks */
  SE2PoseEstimation(const std::vector<Eigen::Vector2d> & landmarks);
  /** Constructor with a given set of landmarks and IDs */
  SE2PoseEstimation(
    const std::vector<Eigen::Vector2d> & landmarks,
    const std::vector<unsigned long> & ids
  );

  /** Add a new landmark to the graph */
  void add_landmark(const Eigen::Vector2d & landmark);
  /** Add a new landmark to the graph with the given ID.
      Returns false if the ID already exists.
  */
  bool add_landmark(const Eigen::Vector2d & landmark, const unsigned long id);
  /** Add a new landmark to the graph from the msg format */
  bool add_landmark(const gpe_msgs::msg::Landmark & landmark_msg);
  /** Add new landmarks to the graph */
  void add_landmarks(const std::vector<Eigen::Vector2d> & landmarks);
  /** Add a new landmarks to the graph with a list of IDs.
     Sizes must match and IDs must be unique.
     This is an atomic operation. All IDs must be valid.
  */
  bool add_landmarks(
    const std::vector<Eigen::Vector2d> & landmarks,
    const std::vector<unsigned long> & ids
  );
  /** Add new landmarks to the graph from the msg format */
  bool add_landmarks(const gpe_msgs::msg::LandmarkArray & landmarks_msg);

  /** Get the current list of landmarks */
  inline std::vector<Eigen::Vector2d> get_landmarks() const {return landmarks_;}

  /** Set the initial pose estimation.
      Calling this function will remove all measurements.
  */
  void set_initial_pose(const g2o::SE2 & initial_pose);

  /** Add a new measurement with its information matrix */
  void add_measurement(
    const Eigen::Vector2d & measurement,
    const Eigen::Matrix2d & inf_matrix
  );

  /** Add a new measurement with its information matrix and the ID of the landmark */
  void add_measurement(
    const Eigen::Vector2d & measurement,
    const Eigen::Matrix2d & inf_matrix,
    const unsigned long landmark_id
  );

  /** Remove all measurements from the graph */
  void reset_measurements();

  /** Run the optimization and return the estimated pose.
      The estimated pose is saved as the initial estimation for the next call.
  */
  g2o::SE2 estimate();

  /** Returns the last estimated pose */
  g2o::SE2 pose() const {return pose_;}

  /** Return a reference to the G2O optimizer object. Use with caution! */
  g2o::SparseOptimizer & get_optimizer() {return optimizer_;}

  /** Check if the ID is already taken by a landmark */
  bool is_landmark_id(const unsigned long id);

private:
  /** Initialize G2O optimizer objects */
  void initialize_optimizer();

  /** Run the association algorithm to find the corresponding
      landmark IDs for the measurements without one
   */
  void associate_detached_measurements();

  /** Increase the ID counter used for the IDs.
      This checks if the next ID is already taken by the user to skip it.
  */
  void increase_node_id();
  /** Changes the ID used for the pose.
      This function is called whenever a new landmark needs the ID used by the pose.
  */
  void update_pose_id();

  // State (map and poses)
  g2o::SE2 pose_;
  std::vector<Eigen::Vector2d> landmarks_;  // TODO: Probably not needed?
  // List of measurements that are not associated yet to any landmark
  std::vector<g2o::EdgeSE2PointXY *> detached_measurements_;

  // G2O graph IDs and lookup tables
  // ID counter used for landmarks. Starts at zero.
  unsigned long node_id_ = 0;
  std::vector<unsigned long> landmark_ids_;
  // ID used for the pose. Defaults to 1000000 but can automatically change
  // if a landmark uses that number.
  unsigned long pose_id_ = 1000000;
  // Optimization objects
  g2o::SparseOptimizer optimizer_;
};

}  // namespace gpe

#endif  // GPE_CORE__SE2_POSE_ESTIMATION_HPP_
