// Copyright 2022 Jacob Hartzer
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef EKF__SENSORS__LIDAR_HPP_
#define EKF__SENSORS__LIDAR_HPP_

#include <string>

#include "ekf/sensors/Sensor.hpp"

///
/// @class Lidar
/// @brief LIDAR Sensor Class
/// @todo Implement update methods
///
class Lidar : public Sensor
{
public:
  ///
  /// @brief Lidar initialization parameters structure
  ///
  typedef struct Params
  {
    std::string name;                                  ///< @brief Lidar name
    double rate{1.0};                                  ///< @brief Lidar update rate
    Eigen::Vector3d posOffset{0.0, 0.0, 0.0};          ///< @brief Lidar initial position offset
    Eigen::Quaterniond angOffset{1.0, 0.0, 0.0, 0.0};  ///< @brief Lidar initial angular offset
    Eigen::VectorXd variance {{0, 0, 0, 0, 0, 0}};     ///< @brief Initial state variance
  } Params;

  ///
  /// @brief Lidar sensor constructor
  /// @param params Lidar sensor parameters
  ///
  explicit Lidar(Lidar::Params params);

  ///
  /// @brief Predict measurement method
  /// @return Predicted measurement vector
  ///
  Eigen::VectorXd PredictMeasurement();

  ///
  /// @brief Measurement Jacobian method
  /// @return Measurement Jacobian matrix
  ///
  Eigen::MatrixXd GetMeasurementJacobian();

  ///
  /// @brief State setter
  /// @param state Input state vector
  ///
  void SetState(Eigen::VectorXd state);

  ///
  /// @brief Sensor state getter method
  /// @return Sensor state vector
  ///
  Eigen::VectorXd GetState();
};

#endif  // EKF__SENSORS__LIDAR_HPP_
