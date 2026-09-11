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

#include <optimizable_graph.h>
#include <vector>
#include <set>
#include <cmath>

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

#include <gpe_msgs/msg/landmark2_d_array.hpp>

#include <gpe_core/types.hpp>

namespace gpe
{

class SE2PoseEstimation
{
public:
  /** Default constructor */
  SE2PoseEstimation();

  /** Constructor with a given set of landmarks */
  SE2PoseEstimation(const LandmarkArray & landmarks_msg);

  /**
   * Add a new landmark to the graph.
   * If the landmark has an unknown ID (-1), it will be given one using node_id_
   * If the landmark has a given positive ID, it will be used in the graph
   *
   * @param landmark_msg The landmark object.
   * @return True if the landmark was successfully added, false if the ID already exists.
   */
  bool add_landmark(const Landmark & landmark_msg);

  /**
   * Add new landmarks to the graph.
   * This operation is atomic, all landmarks are added, or none.
   *
   * @param landmark_msg The landmark object.
   * @return True if the landmark was successfully added.
             False if any of the IDs already exists or if there are duplicate IDs.
   */
  bool add_landmarks(const LandmarkArray & landmarks_msg);

  /**
   * Get the current list of landmarks
   * @return A Landmark2DArray object with the list of landmarks
   */
  LandmarkArray get_landmarks() const;

  /**
   * Set the initial pose estimation.
   * Calling this function will remove all measurements.
   *
   * @param initial_pose The estimate for the current robot pose
   */
  void set_initial_pose(const g2o::SE2 & initial_pose);

  /**
   * Add a new measurement to a landmark.
   * The measurement can be of any of the types included in the Measurement variant.
   * The measurement must include its information matrix.
   *
   * @param measurement The Measurement variant to add
   */
  void add_measurement(const Measurement & measurement);

  /**
   * Add a new measurement with its information matrix and the ID of the landmark.
   * The measurement can be of any of the types included in the Measurement variant.
   * The measurement must include its information matrix.
   *
   * @param measurement The Measurement variant to add
   * @param landmark_id The ID of the landmark to associate the measurement with
   * @return false if the landmark_id does not exist in the graph
             or if the measurement's edge already exists
   */
  bool add_measurement(const Measurement & measurement, const unsigned int landmark_id);

  /** Remove all measurements from the graph */
  void reset_measurements();

  /**
   * Run the optimization on the graph and return the estimated pose.
   * The estimated pose is saved as the initial estimation for the next call.
   * @return The estimated pose as a g2o::SE2 object.
   */
  g2o::SE2 estimate();

  /**
   * Run the Kabsch algorithm using the graph data and return the estimated pose.
   * The estimated pose is saved as the initial estimation for the next call.
   *
   * This method exists purely as a comparison against the g2o optimizer and should not be used.
   * It will reuse the same measurements and associations (hungarian method) from the graph,
   * then the Kabsch algorithm will be applied to estimate the pose.
   * Information matrices are not used directly with Kabsch,
   * but the Hungarian method considers them.
   *
   * NOTE: This method will ONLY use measurements with type MeasurementXY (only points).
   *       It does not make sense to perform Kabsch with MeasurementSE2, so they are ignored.
   * @return The estimated pose as a g2o::SE2 object.
   */
  g2o::SE2 estimate_kabsch();


  /**
   * Estimate the robot pose by averaging the estimations from SE2 measurements.
   * The estimated pose is saved as the initial estimation for the next call.
   *
   * This method exists purely as a comparison against the g2o optimizer and should not be used.
   * It will reuse the same measurements and associations (hungarian method) from the graph.
   *
   * Then, each MeasurementSE2 is used to estimate the robot pose.
   * The final pose estimation is the weighted average of all the individual estimations.
   * The weights are given by the chi-squared error computed by G2O.
   *
   * NOTE: This method will ONLY use measurements with type MeasurementSE2 (only poses).
   *       It does not make sense to perform average estimation with MeasurementXY
   *       (not enough info), so they are ignored.
   * @return The estimated pose as a g2o::SE2 object.
   */
  g2o::SE2 estimate_pose_avg();

  /** Returns the last estimated pose */
  g2o::SE2 pose() const {return pose_;}

  /** Return a reference to the G2O optimizer object. Use with caution! */
  g2o::SparseOptimizer & get_optimizer() {return optimizer_;}

private:
  /** Initialize G2O optimizer objects */
  void initialize_optimizer();

  /**
   * Run the association algorithm to find the corresponding
   * landmark IDs for the measurements without one
   * This method is templated to select the type of measurements (MeasurementXY / MeasurementSE2)
   * The association must be done separately for each type
   *
   * @tparam M The type of measurement.
   */
  template<typename M>
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

  // List of measurements that are not associated yet to any landmark
  std::vector<g2o::OptimizableGraph::Edge *> detached_measurements_xy_;
  std::vector<g2o::OptimizableGraph::Edge *> detached_measurements_se2_;

  /**
   * Private getter to access each of the different detached measurements list based on a template
   *
   * @tparam M The type of mesaurement. Only specialized for MeasurementXY / MeasurementSE2
   * @return a reference to the detached measurements vector
   */
  template<typename M>
  std::vector<g2o::OptimizableGraph::Edge *> & get_detached_measurement();

  // G2O graph IDs and lookup tables
  // ID counter used for landmarks. Starts at zero.
  unsigned int node_id_ = 0;
  std::set<unsigned int> landmark_ids_;
  // ID used for the pose. Defaults to 1000000 but can automatically change
  // if a landmark uses that number.
  unsigned int pose_id_ = 1000000;
  // Optimization objects
  g2o::SparseOptimizer optimizer_;
};

}  // namespace gpe

#endif  // GPE_CORE__SE2_POSE_ESTIMATION_HPP_
