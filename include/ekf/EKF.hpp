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

#ifndef EKF__EKF_HPP_
#define EKF__EKF_HPP_

#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#include "ekf/sensors/Camera.hpp"
#include "ekf/sensors/Imu.hpp"
#include "ekf/sensors/Lidar.hpp"
#include "ekf/sensors/Sensor.hpp"

///
/// @class EKF
/// @brief Calibration EKF class
///
class EKF
{
public:
  ///
  /// @brief EKF class constructor
  ///
  EKF();

  ///
  /// @brief IMU message callback method
  ///
  void ImuCallback(
    unsigned int id, double time,
    Eigen::Vector3d acceleration,
    Eigen::Matrix3d accelerationCovariance,
    Eigen::Vector3d angularRate,
    Eigen::Matrix3d angularRateCovariance);

  ///
  /// @brief Camera message callback method
  /// @param id Camera sensor ID
  /// @param time Camera measurement time
  ///
  // void CameraCallback(unsigned int id, double time);
  void CameraCallback();

  ///
  /// @brief Lidar message callback method
  /// @param id Lidar sensor ID
  /// @param time Lidar measurement time
  ///
  // void LidarCallback(unsigned int id, double time);
  void LidarCallback();

  ///
  /// @brief Registers new IMU sensor to calibration EKF
  /// @param params Sensor parameters to use in registration
  ///
  unsigned int RegisterSensor(typename Imu::Params params);

  ///
  /// @brief Registers new Camera sensor to calibration EKF
  /// @param params Sensor parameters to use in registration
  ///
  unsigned int RegisterSensor(typename Camera::Params params);

  ///
  /// @brief Registers new LIDAR sensor to calibration EKF
  /// @param params Sensor parameters to use in registration
  ///
  unsigned int RegisterSensor(typename Lidar::Params params);

  ///
  /// @brief Getter method for state vector
  /// @return State vector
  ///
  Eigen::VectorXd GetState();

  ///
  /// @brief Getter method for state covariance matrix
  /// @return State covariance matrix
  ///
  Eigen::MatrixXd GetCov();

  ///
  /// @brief Getter method for state size
  /// @return State size
  ///
  unsigned int GetStateSize();

  ///
  /// @brief Predict state to given time
  /// @param currentTime Time for prediction
  ///
  void Predict(double currentTime);

  ///
  /// @brief State transition matrix getter method
  /// @param dT State transition time
  /// @return State transition matrix
  ///
  Eigen::MatrixXd GetStateTransition(double dT);

  ///
  /// @brief Process input matrix getter method
  /// @return Process input matrix
  ///
  Eigen::MatrixXd GetProcessInput();

  ///
  /// @brief Process noise matrix getter method
  /// @return Process noise matrix
  ///
  Eigen::MatrixXd GetProcessNoise();

  ///
  /// @brief EKF state initialization method
  /// @param timeInit Initial time
  /// @param bodyStateInit Initial state
  ///
  void InitializeBodyState(double timeInit, Eigen::VectorXd bodyStateInit);

  void GetTransforms(
    std::string & baseImuName,
    std::vector<std::string> & sensorNames,
    std::vector<Eigen::Vector3d> & sensorPosOffsets,
    std::vector<Eigen::Quaterniond> & sensorAngOffsets
  );

private:
  void ExtendState(
    unsigned int sensorStateSize, Eigen::VectorXd sensorState,
    Eigen::MatrixXd sensorCov);

  unsigned int m_stateSize{18U};
  Eigen::VectorXd m_state = Eigen::VectorXd::Zero(18U);
  Eigen::MatrixXd m_cov = Eigen::MatrixXd::Identity(18U, 18U);
  std::map<int, std::shared_ptr<Imu>> m_mapImu{};
  std::map<int, std::shared_ptr<Camera>> m_mapCamera{};
  std::map<int, std::shared_ptr<Lidar>> m_mapLidar{};
  double m_currentTime {0};
  bool m_timeInitialized {false};
};

#endif  // EKF__EKF_HPP_
