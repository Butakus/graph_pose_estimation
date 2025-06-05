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

#ifndef GPE_CORE__TYPES_HPP_
#define GPE_CORE__TYPES_HPP_

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <g2o/types/slam2d/se2.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/edge_se2_pointxy.h>

#include <gpe_msgs/msg/landmark2_d_array.hpp>

namespace gpe
{

using Landmark = gpe_msgs::msg::Landmark2D;
using LandmarkArray = gpe_msgs::msg::Landmark2DArray;


// TODO: Not used yet? Maybe this is useless
enum class LandmarkType2D
{
  PointXY,
  PoseSE2,
};

// enum class LandmarkType3D
// {
//   PointXYZ,
//   PoseSE3,
// };

/**
 * A 2D measurement from an SE2 pose to a XY point.
 * Data is stored as a Eigen::Vector2d, information matrix as a Eigen::Matrix2D
 * In the graph it is represented as a g2o::EdgeSE2PointXY
 */
class MeasurementXY
{
public:
  using EdgeType = g2o::EdgeSE2PointXY;

  MeasurementXY()
  : inf_matrix(Eigen::Matrix2d::Identity())
  {}

  MeasurementXY(Eigen::Vector2d measurement)
  : data(std::move(measurement)),
    inf_matrix(Eigen::Matrix2d::Identity())
  {}

  MeasurementXY(
    Eigen::Vector2d measurement,
    Eigen::Matrix2d inf_matrix
  )
  : data(std::move(measurement)),
    inf_matrix(std::move(inf_matrix))
  {}

  // TODO: Add constructor from vision_msgs detection

  [[nodiscard]] double x() {return data.x();}
  [[nodiscard]] double y() {return data.y();}
  void set_x(double x) {data.x() = x;}
  void set_y(double y) {data.y() = y;}

  Eigen::Vector2d data {};
  Eigen::Matrix2d inf_matrix {};
};

/**
 * A 2D measurement from an SE2 pose to an SE2 pose.
 * Data is stored as a g2o::SE2, information matrix as a Eigen::Matrix3d
 * In the graph it is represented as a g2o::EdgeSE2
 */
class MeasurementSE2
{
public:
  using EdgeType = g2o::EdgeSE2;

  MeasurementSE2()
  : inf_matrix(Eigen::Matrix3d::Identity())
  {}

  MeasurementSE2(g2o::SE2 measurement)
  : data(std::move(measurement)),
    inf_matrix(Eigen::Matrix3d::Identity())
  {}

  MeasurementSE2(
    g2o::SE2 measurement,
    Eigen::Matrix3d inf_matrix
  )
  : data(std::move(measurement)),
    inf_matrix(std::move(inf_matrix))
  {}

  // TODO: Add constructor from vision_msgs detection

  [[nodiscard]] double x() {return data.translation().x();}
  [[nodiscard]] double y() {return data.translation().y();}
  [[nodiscard]] double theta() {return data.rotation().angle();}
  void set_x(double new_x) {data.setTranslation({new_x, y()});}
  void set_y(double new_y) {data.setTranslation({x(), new_y});}
  void set_theta(double new_theta) {data.setRotation(Eigen::Rotation2D(new_theta));}

  g2o::SE2 data {};
  Eigen::Matrix3d inf_matrix {};
};

}  // namespace gpe

#endif  // GPE_CORE__TYPES_HPP_
