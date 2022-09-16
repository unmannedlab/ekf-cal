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

#include "ekf/EKF.hpp"

#include <memory>

#include "ekf/sensors/Imu.hpp"

EKF::EKF() {}

unsigned int EKF::RegisterSensor(typename Imu::Params params)
{
  std::shared_ptr<Imu> sensor_ptr = std::make_shared<Imu>(params);
  m_mapImu[sensor_ptr->GetId()] = sensor_ptr;
  sensor_ptr->SetStateStartIndex(m_stateSize);

  if (sensor_ptr->GetStateSize() != 0) {
    ExtendState(sensor_ptr->GetStateSize(), sensor_ptr->GetState(), sensor_ptr->GetCov());
  }
  return sensor_ptr->GetId();
}

unsigned int EKF::RegisterSensor(typename Camera::Params params)
{
  std::shared_ptr<Camera> sensor_ptr = std::make_shared<Camera>(params);
  m_mapCamera[sensor_ptr->GetId()] = sensor_ptr;
  sensor_ptr->SetStateStartIndex(m_stateSize);

  if (sensor_ptr->GetStateSize() != 0) {
    ExtendState(sensor_ptr->GetStateSize(), sensor_ptr->GetState(), sensor_ptr->GetCov());
  }

  return sensor_ptr->GetId();
}

unsigned int EKF::RegisterSensor(typename Lidar::Params params)
{
  std::shared_ptr<Lidar> sensor_ptr = std::make_shared<Lidar>(params);
  m_mapLidar[sensor_ptr->GetId()] = sensor_ptr;
  sensor_ptr->SetStateStartIndex(m_stateSize);

  if (sensor_ptr->GetStateSize() != 0) {
    ExtendState(sensor_ptr->GetStateSize(), sensor_ptr->GetState(), sensor_ptr->GetCov());
  }

  return sensor_ptr->GetId();
}

void EKF::ExtendState(
  unsigned int sensorStateSize, Eigen::VectorXd sensorState,
  Eigen::MatrixXd sensorCov)
{
  // Resize State and Covariance
  m_state.conservativeResize(m_stateSize + sensorStateSize);
  m_cov.conservativeResize(m_stateSize + sensorStateSize, m_stateSize + sensorStateSize);

  // Set new states to zero
  m_state.segment(m_stateSize, sensorStateSize) = sensorState;

  // Set new covariance to identity and cross-covariance to zero
  m_cov.block(m_stateSize, m_stateSize, sensorStateSize, sensorStateSize) = sensorCov;
  m_cov.block(0, m_stateSize, m_stateSize, sensorStateSize).setZero();
  m_cov.block(m_stateSize, 0, sensorStateSize, m_stateSize).setZero();

  m_stateSize += sensorStateSize;
}

Eigen::MatrixXd EKF::GetStateTransition(double dT)
{
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(m_stateSize, m_stateSize);
  F.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3) * dT;
  F.block<3, 3>(3, 6) = Eigen::MatrixXd::Identity(3, 3) * dT;
  F.block<3, 3>(9, 12) = Eigen::MatrixXd::Identity(3, 3) * dT;
  F.block<3, 3>(12, 15) = Eigen::MatrixXd::Identity(3, 3) * dT;
  return F;
}

Eigen::MatrixXd EKF::GetProcessInput()
{
  Eigen::MatrixXd G = Eigen::MatrixXd::Identity(m_stateSize, m_stateSize);
  return G;
}

Eigen::MatrixXd EKF::GetProcessNoise()
{
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(m_stateSize, m_stateSize);
  unsigned int stateStart {0};
  double accBiasStability {0};
  double omgBiasStability {0};

  for (auto const & iter : m_mapImu) {
    if (iter.second->IsIntrinsic()) {
      stateStart = iter.second->GetStateStartIndex();
      accBiasStability = iter.second->GetAccBiasStability();
      omgBiasStability = iter.second->GetOmgBiasStability();

      Q.block<3, 3>(
        stateStart + 6,
        stateStart + 6) = Eigen::MatrixXd::Identity(3, 3) * accBiasStability;
      Q.block<3, 3>(
        stateStart + 9,
        stateStart + 9) = Eigen::MatrixXd::Identity(3, 3) * omgBiasStability;
    }
  }

  return Q;
}

void EKF::Predict(double time)
{
  // Don't predict if time is not initialized
  if (!m_timeInitialized) {
    m_currentTime = time;
    m_timeInitialized = true;
    return;
  }

  if (time < m_currentTime) {
    /// @todo replace with generic logging
    // RCLCPP_WARN(rclcpp::get_logger("EKF"), "Requested time in the past");
    return;
  }
  double dT = time - m_currentTime;

  Eigen::MatrixXd F = GetStateTransition(dT);
  Eigen::MatrixXd G = GetProcessInput();
  Eigen::MatrixXd Q = GetProcessNoise();

  /// @todo Should create convolution function to handle quaternion multiplication
  m_state = F * m_state;
  m_cov = F * m_cov * F.transpose() + F * G * Q * G.transpose() * F.transpose();
  m_currentTime = time;
  Sensor::SetBodyState(m_state.segment<18>(0));
}

void EKF::ImuCallback(
  unsigned int id, double time, Eigen::Vector3d acceleration,
  Eigen::Matrix3d accelerationCovariance, Eigen::Vector3d angularRate,
  Eigen::Matrix3d angularRateCovariance)
{
  auto iter = m_mapImu.find(id);
  /// @todo replace with generic logging
  // RCLCPP_INFO(
  //   rclcpp::get_logger("EKF"), "IMU Callback: '%s', '%f'",
  //   iter->second->GetName().c_str(), time);
  Predict(time);

  Eigen::VectorXd z(acceleration.size() + angularRate.size());
  z.segment<3>(0) = acceleration;
  z.segment<3>(3) = angularRate;

  Eigen::VectorXd z_pred = iter->second->PredictMeasurement();
  Eigen::VectorXd resid = z - z_pred;

  unsigned int stateSize = iter->second->GetStateSize();
  unsigned int stateStartIndex = iter->second->GetStateStartIndex();
  Eigen::MatrixXd subH = iter->second->GetMeasurementJacobian();
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, m_stateSize);
  H.block<6, 18>(0, 0) = subH.block<6, 18>(0, 0);
  H.block(0, stateStartIndex, 6, stateSize) = subH.block(0, 18, 6, stateSize);

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(6, 6);
  R.block<3, 3>(0, 0) = accelerationCovariance;
  R.block<3, 3>(3, 3) = angularRateCovariance;
  for (int i = 0; i < 3; ++i) {
    if (R(i, i) < 1e-3) {
      R(i, i) = 1e-3;
    }
  }
  for (int i = 3; i < 6; ++i) {
    if (R(i, i) < 1e-2) {
      R(i, i) = 1e-2;
    }
  }

  Eigen::MatrixXd S = H * m_cov * H.transpose() + R;
  Eigen::MatrixXd K = m_cov * H.transpose() * S.inverse();

  m_state = m_state + K * resid;
  m_cov = (Eigen::MatrixXd::Identity(m_stateSize, m_stateSize) - K * H) * m_cov;

  // Only set state if nonzero in size
  if (iter->second->GetStateSize() > 0) {
    iter->second->SetState(m_state.segment(stateStartIndex, stateSize));
  }

  Sensor::SetBodyState(m_state.segment<18>(0));
}

void EKF::CameraCallback()
// void EKF::CameraCallback(unsigned int id, double time)
{
  // RCLCPP_WARN(
  //   rclcpp::get_logger("EKF"),
  //   "Camera callback for '%u' at '%g' not implemented", id, time);
}

void EKF::LidarCallback()
// void EKF::LidarCallback(unsigned int id, double time)
{
  // RCLCPP_WARN(
  //   rclcpp::get_logger("EKF"),
  //   "Lidar callback for '%u' at '%g' not implemented", id, time);
}

Eigen::VectorXd EKF::GetState()
{
  return m_state;
}

Eigen::MatrixXd EKF::GetCov()
{
  return m_cov;
}

unsigned int EKF::GetStateSize()
{
  return m_stateSize;
}


void EKF::GetTransforms(
  std::string & baseImuName,
  std::vector<std::string> & sensorNames, std::vector<Eigen::Vector3d> & sensorPosOffsets,
  std::vector<Eigen::Quaterniond> & sensorAngOffsets)
{
  // Iterate over IMUs
  for (auto const & iter : m_mapImu) {
    if (iter.second->IsBaseSensor()) {
      baseImuName = iter.second->GetName();
    } else {
      sensorNames.push_back(iter.second->GetName());
      sensorPosOffsets.push_back(iter.second->GetPosOffset());
      sensorAngOffsets.push_back(iter.second->GetAngOffset());
    }
  }
  // Iterate over Cameras
  for (auto const & iter : m_mapCamera) {
    sensorNames.push_back(iter.second->GetName());
    sensorPosOffsets.push_back(iter.second->GetPosOffset());
    sensorAngOffsets.push_back(iter.second->GetAngOffset());
  }
  // Iterate over LiDARs
  for (auto const & iter : m_mapLidar) {
    sensorNames.push_back(iter.second->GetName());
    sensorPosOffsets.push_back(iter.second->GetPosOffset());
    sensorAngOffsets.push_back(iter.second->GetAngOffset());
  }
}


void EKF::InitializeBodyState(double timeInit, Eigen::VectorXd bodyStateInit)
{
  m_currentTime = timeInit;
  m_timeInitialized = true;
  m_state.segment<18>(0) = bodyStateInit;
  Sensor::SetBodyState(bodyStateInit);
}
