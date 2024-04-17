#ifndef GPE_CORE__SE2_POSE_ESTIMATION_HPP_
#define GPE_CORE__SE2_POSE_ESTIMATION_HPP_

#include "gpe_core/visibility_control.h"

#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <iostream>
#include <stdint.h>
#include <vector>
#include <cmath>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam2d/se2.h>
#include <g2o/types/slam2d/vertex_point_xy.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2_pointxy.h>

namespace gpe_core
{

class SE2PoseEstimation
{
public:
  SE2PoseEstimation();

  virtual ~SE2PoseEstimation();
};

}  // namespace gpe_core

#endif  // GPE_CORE__SE2_POSE_ESTIMATION_HPP_
