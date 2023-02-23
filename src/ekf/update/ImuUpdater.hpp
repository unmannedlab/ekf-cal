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

#ifndef EKF__UPDATE__IMUUPDATER_HPP_
#define EKF__UPDATE__IMUUPDATER_HPP_

#include "ekf/EKF.hpp"
#include "ekf/update/Updater.hpp"

///
/// @class ImuUpdater
/// @brief EKF Updater Class for IMU Sensors
///
class ImuUpdater : public Updater
{
public:
  ///
  /// @brief IMU EKF Updater Constructor
  /// @param imuID IMU Sensor ID
  /// @param accBiasStability
  /// @param omgBiasStability
  ///
  ImuUpdater(
    unsigned int imuID,
    double accBiasStability,
    double omgBiasStability
  );


  ///
  /// @brief Predict measurement method
  /// @return Predicted measurement vector
  /// @todo Add gravity to prediction
  ///
  Eigen::VectorXd predictMeasurement();

  ///
  /// @brief Measurement Jacobian method
  /// @return Measurement Jacobian matrix
  ///
  Eigen::MatrixXd getMeasurementJacobian();

  ///
  /// @brief EKF update method for IMU measurements
  /// @param time Measurement time
  /// @param acceleration Measured acceleration
  /// @param accelerationCovariance Estimated acceleration error
  /// @param angularRate Measured angular rate
  /// @param angularRateCovariance Estimated angular rate error
  ///
  void updateEKF(
    double time, Eigen::Vector3d acceleration, Eigen::Matrix3d accelerationCovariance,
    Eigen::Vector3d angularRate, Eigen::Matrix3d angularRateCovariance);

  ///
  /// @brief Refresh internal states with EKF values
  ///
  void RefreshStates();

private:
  static const Eigen::Vector3d GRAVITY;
  Eigen::Vector3d m_bodyPos {0.0, 0.0, 0.0};
  Eigen::Vector3d m_bodyVel {0.0, 0.0, 0.0};
  Eigen::Vector3d m_bodyAcc {0.0, 0.0, 0.0};
  Eigen::Quaterniond m_bodyAngPos {1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d m_bodyAngVel {0.0, 0.0, 0.0};
  Eigen::Vector3d m_bodyAngAcc {0.0, 0.0, 0.0};
  Eigen::Vector3d m_posOffset {0.0, 0.0, 0.0};
  Eigen::Quaterniond m_angOffset {1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d m_accBias {0.0, 0.0, 0.0};
  Eigen::Vector3d m_omgBias {0.0, 0.0, 0.0};
  double m_accBiasStability;
  double m_omgBiasStability;
};

#endif  // EKF__UPDATE__IMUUPDATER_HPP_