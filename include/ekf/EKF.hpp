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
#include <vector>

#include "ekf/sensors/Camera.hpp"
#include "ekf/sensors/Imu.hpp"
#include "ekf/sensors/Lidar.hpp"
#include "ekf/sensors/Sensor.hpp"

class EKF
{
public:
  ///
  /// @class EKF
  /// @brief
  /// @todo  Literally everything
  ///
  EKF();

  // void RegisterIMU(IMU::Params params);
  // void RegisterCamera(Camera::Params params);
  // void RegisterLIDAR(LIDAR::Params params);

  void ImuCallback();
  void CameraCallback();
  void LidarCallback();

  template<typename T>
  unsigned int RegisterSensor(typename T::Params params)
  {
    T sensor(params);
    m_sensorList.push_back(sensor);
    return sensor.GetId();
  }

private:
  void Predict(double currentTime);
  unsigned int m_stateSize{0};
  Eigen::VectorXd m_state = Eigen::VectorXd::Zero(18U);
  Eigen::MatrixXd m_cov = Eigen::MatrixXd::Zero(18U, 18U);
  std::vector<Sensor> m_sensorList{};
};

#endif  // EKF__EKF_HPP_