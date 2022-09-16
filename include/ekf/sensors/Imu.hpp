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

#ifndef EKF__SENSORS__IMU_HPP_
#define EKF__SENSORS__IMU_HPP_

#include <string>

#include "ekf/sensors/Sensor.hpp"

///
/// @class Imu
/// @brief IMU Sensor Class
/// @todo Add parameter input/defaults for covariance
///
class Imu : public Sensor
{
public:
  ///
  /// @brief Imu initialization parameters structure
  ///
  typedef struct Params
  {
    std::string name;                               ///< @brief IMU name
    bool baseSensor{false};                         ///< @brief IMU base sensor flag
    bool intrinsic{false};                          ///< @brief IMU intrinsic calibration
    double rate{1.0};                               ///< @brief IMU update rate
    Eigen::Vector3d posOffset {0, 0, 0};            ///< @brief IMU position offset vector
    Eigen::Quaterniond angOffset {1, 0, 0, 0};      ///< @brief IMU angular offset quaternion
    Eigen::Vector3d accBias {0, 0, 0};              ///< @brief IMU accelerometer bias vector
    Eigen::Vector3d omgBias {0, 0, 0};              ///< @brief IMU gyroscope bias vector
    double accBiasStability {0};                    ///< @brief IMU accelerometer bias stability
    double omgBiasStability {0};                    ///< @brief IMU gyroscope bias stability
    /// @brief Initial state variance
    Eigen::VectorXd variance {{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
  } Params;

  ///
  /// @brief IMU class constructor
  /// @param params IMU sensor parameters
  ///
  explicit Imu(Imu::Params params);

  ///
  /// @brief Predict measurement method
  /// @return Predicted measurement vector
  /// @todo Add gravity to prediction
  ///
  Eigen::VectorXd PredictMeasurement();

  ///
  /// @brief Measurement Jacobian method
  /// @return Measurement Jacobian matrix
  ///
  Eigen::MatrixXd GetMeasurementJacobian();

  ///
  /// @brief Accelerometer bias stability getter method
  /// @return Accelerometer bias stability
  ///
  double GetAccBiasStability();

  ///
  /// @brief Gyroscope bias stability getter method
  /// @return Gyroscope bias stability
  ///
  double GetOmgBiasStability();

  ///
  /// @brief Base sensor flag getter method
  /// @return Base sensor flag
  ///
  bool IsBaseSensor();

  ///
  /// @brief Intrinsic sensor flag getter method
  /// @return Intrinsic sensor flag
  ///
  bool IsIntrinsic();

  ///
  /// @brief Sensor state setter method
  /// @param state Sensor state vector
  ///
  void SetState(Eigen::VectorXd state);

  ///
  /// @brief Sensor state getter method
  /// @return Sensor state vector
  ///
  Eigen::VectorXd GetState();

private:
  bool m_baseSensor;
  bool m_intrinsic;
  Eigen::Vector3d m_posOffset;
  Eigen::Quaterniond m_angOffset;
  Eigen::Vector3d m_accBias;
  Eigen::Vector3d m_omgBias;
  double m_accBiasStability;
  double m_omgBiasStability;
};

#endif  // EKF__SENSORS__IMU_HPP_
