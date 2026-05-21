// Copyright 2026 Francisco Miguel Moreno
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
// This file implements the Kabsch algorithm for estimating the optimal rotation and translation
// between two sets of corresponding 2D/3D points.
// The implementation is partially based on this document by Olga Sorkine:
//   https://igl.ethz.ch/projects/ARAP/svd_rot.pdf

#ifndef GPE_CORE__KABSCH_ALIGNMENT_HPP_
#define GPE_CORE__KABSCH_ALIGNMENT_HPP_

#include <Eigen/Dense>
#include <cstddef>

namespace gpe
{

/**
 * Estimate the optimal rotation and translation between two sets of corresponding points using the Kabsch algorithm.
 * @tparam N The dimension of the points (2 for 2D, 3 for 3D).
 * @param measured_points The first set of points (e.g., measurements). Each row is a point.
 * @param landmark_points The second set of points (e.g., landmarks). Each row is a point.
 * @return The optimal homogeneous transformation matrix from landmark_points to measured_points.
 */
template<size_t N>
Eigen::Matrix<double, N + 1, N + 1>
kabsch_alignment(
  const Eigen::Matrix<double, N, Eigen::Dynamic> & measured_points,
  const Eigen::Matrix<double, N, Eigen::Dynamic> & landmark_points
)
{
  // Generic type aliases
  using MatrixN = Eigen::Matrix<double, N, N>;
  using VectorN = Eigen::Matrix<double, N, 1>;

  // 1. Compute centroids
  const VectorN centroid_measured = measured_points.rowwise().mean();
  const VectorN centroid_landmark = landmark_points.rowwise().mean();

  // 2. Center the data
  const auto centered_measured = measured_points.colwise() - centroid_measured;
  const auto centered_landmark = landmark_points.colwise() - centroid_landmark;

  // 3. Compute covariance
  const MatrixN covariance = centered_measured * centered_landmark.transpose();

  // 4. Compute SVD
  Eigen::JacobiSVD<MatrixN> svd(
    covariance,
    Eigen::ComputeFullU | Eigen::ComputeFullV
  );

  // 5. Find optimal rotation (reflection-safe)
  const MatrixN u = svd.matrixU();
  const MatrixN v = svd.matrixV();
  MatrixN reflection = MatrixN::Identity();
  reflection(1, 1) = (v * u.transpose()).determinant();
  const MatrixN rotation = v * reflection * u.transpose();

  // 6. Find translation
  const VectorN translation = centroid_landmark - rotation * centroid_measured;

  // 7. Build homogeneous transformation matrix
  Eigen::Matrix<double, N + 1, N + 1> transformation =
    Eigen::Matrix<double, N + 1, N + 1>::Identity();
  transformation.template block<N, N>(0, 0) = rotation;
  transformation.template block<N, 1>(0, N) = translation;

  return transformation;
}

}  // namespace gpe

#endif  // GPE_CORE__KABSCH_ALIGNMENT_HPP_
