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

#include <stdexcept>
#include <variant>

#include <Eigen/Dense>
#include <g2o/types/slam2d/types_slam2d.h>

#include <gpe_msgs/msg/landmark2_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>

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
  using VertexType = g2o::VertexPointXY;

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

  // Constructor from vision_msgs detection
  MeasurementXY(const vision_msgs::msg::Detection3D & detecion_msg)
  : inf_matrix(Eigen::Matrix2d::Identity())
  {
    // There must be at least 1 detectio hypothesis. Only the first one will be used
    if (detecion_msg.results.size() == 0) {
      throw std::invalid_argument("Detection2D message has an empty list of hypothesis");
    }
    data.x() = detecion_msg.results[0].pose.pose.position.x;
    data.y() = detecion_msg.results[0].pose.pose.position.y;
    Eigen::Matrix2d cov_matrix;
    cov_matrix(0, 0) = detecion_msg.results[0].pose.covariance[0];
    cov_matrix(0, 1) = detecion_msg.results[0].pose.covariance[1];
    cov_matrix(1, 0) = detecion_msg.results[0].pose.covariance[6];
    cov_matrix(1, 1) = detecion_msg.results[0].pose.covariance[7];
    inf_matrix = cov_matrix.inverse();
  }


  [[nodiscard]] double x() const {return data.x();}
  [[nodiscard]] double y() const {return data.y();}
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
  using VertexType = g2o::VertexSE2;

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

  // Constructor from vision_msgs detection
  MeasurementSE2(const vision_msgs::msg::Detection3D & detecion_msg)
  : inf_matrix(Eigen::Matrix3d::Identity())
  {
    // There must be at least 1 detection hypothesis. Only the first one will be used
    if (detecion_msg.results.size() == 0) {
      throw std::invalid_argument("Detection2D message has an empty list of hypothesis");
    }
    data.setTranslation({
        detecion_msg.results[0].pose.pose.position.x,
        detecion_msg.results[0].pose.pose.position.y
    });
    Eigen::Matrix3d cov_matrix;
    cov_matrix(0, 0) = detecion_msg.results[0].pose.covariance[0];
    cov_matrix(0, 1) = detecion_msg.results[0].pose.covariance[1];
    cov_matrix(0, 2) = detecion_msg.results[0].pose.covariance[5];
    cov_matrix(1, 0) = detecion_msg.results[0].pose.covariance[6];
    cov_matrix(1, 1) = detecion_msg.results[0].pose.covariance[7];
    cov_matrix(1, 2) = detecion_msg.results[0].pose.covariance[11];
    cov_matrix(2, 0) = detecion_msg.results[0].pose.covariance[30];
    cov_matrix(2, 1) = detecion_msg.results[0].pose.covariance[31];
    cov_matrix(2, 2) = detecion_msg.results[0].pose.covariance[35];
    inf_matrix = cov_matrix.inverse();
  }

  [[nodiscard]] double x() const {return data.translation().x();}
  [[nodiscard]] double y() const {return data.translation().y();}
  [[nodiscard]] double theta() const {return data.rotation().angle();}
  void set_x(double new_x) {data.setTranslation({new_x, y()});}
  void set_y(double new_y) {data.setTranslation({x(), new_y});}
  void set_theta(double new_theta) {data.setRotation(Eigen::Rotation2D(new_theta));}

  g2o::SE2 data {};
  Eigen::Matrix3d inf_matrix {};
};

// A variant type union to hold together measurement types
using Measurement = std::variant<MeasurementXY, MeasurementSE2>;

// A type alias to access the Measurement's Edge inner type
template<typename T>
using EdgeType = T::EdgeType;

// A type alias to access the Measurement's endpoint Vertex inner type
template<typename T>
using VertexType = T::VertexType;

}  // namespace gpe

#endif  // GPE_CORE__TYPES_HPP_
