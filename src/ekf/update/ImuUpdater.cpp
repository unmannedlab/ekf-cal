// Copyright 2023 Jacob Hartzer
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

#include "ekf/update/ImuUpdater.hpp"

#include "ekf/Types.hpp"
#include "infrastructure/Logger.hpp"
#include "sensors/Sensor.hpp"
#include "utility/MathHelper.hpp"
#include "utility/TypeHelper.hpp"

const Eigen::Vector3d ImuUpdater::GRAVITY = Eigen::Vector3d(0, 0, -9.80665);

ImuUpdater::ImuUpdater(unsigned int imuID, double accBiasStability, double omgBiasStability)
: Updater(imuID), m_accBiasStability(accBiasStability), m_omgBiasStability(omgBiasStability) {}

Eigen::VectorXd ImuUpdater::predictMeasurement()
{
  Eigen::VectorXd predictedMeasurement(6);
  // Transform acceleration to IMU location
  Eigen::Vector3d imuAcc = m_bodyAcc +
    m_bodyAngAcc.cross(m_posOffset) +
    m_bodyAngVel.cross((m_bodyAngVel.cross(m_posOffset)));

  // Rotate measurements in place
  Eigen::Vector3d imuAccRot = m_angOffset * imuAcc;
  Eigen::Vector3d imuOmgRot = m_angOffset * m_bodyAngVel;

  // Add bias
  imuAccRot += m_accBias;
  imuOmgRot += m_omgBias;

  predictedMeasurement.segment<3>(0) = imuAccRot;
  predictedMeasurement.segment<3>(3) = imuOmgRot;

  return predictedMeasurement;
}

Eigen::MatrixXd ImuUpdater::getMeasurementJacobian()
{
  Eigen::MatrixXd measurementJacobian(6, 12 + 18);
  measurementJacobian.setZero();

  // Body Acceleration
  measurementJacobian.block<3, 3>(0, 6) = m_angOffset.toRotationMatrix();

  // Body Angular Velocity
  Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(3, 3);
  temp(0, 0) = m_posOffset(1) * m_bodyAngVel(1) + 1 * m_posOffset(2) * m_bodyAngVel(2);
  temp(0, 1) = m_posOffset(1) * m_bodyAngVel(0) - 2 * m_posOffset(0) * m_bodyAngVel(1);
  temp(0, 2) = m_posOffset(2) * m_bodyAngVel(0) - 2 * m_posOffset(0) * m_bodyAngVel(2);
  temp(1, 0) = m_posOffset(0) * m_bodyAngVel(1) - 2 * m_posOffset(1) * m_bodyAngVel(0);
  temp(1, 1) = m_posOffset(0) * m_bodyAngVel(0) + 1 * m_posOffset(2) * m_bodyAngVel(2);
  temp(1, 2) = m_posOffset(2) * m_bodyAngVel(1) - 2 * m_posOffset(1) * m_bodyAngVel(2);
  temp(2, 0) = m_posOffset(0) * m_bodyAngVel(2) - 2 * m_posOffset(2) * m_bodyAngVel(0);
  temp(2, 1) = m_posOffset(1) * m_bodyAngVel(2) - 2 * m_posOffset(2) * m_bodyAngVel(1);
  temp(2, 2) = m_posOffset(0) * m_bodyAngVel(0) + 1 * m_posOffset(1) * m_bodyAngVel(1);
  measurementJacobian.block<3, 3>(0, 12) = m_angOffset * temp;

  // Body Angular Acceleration
  measurementJacobian.block<3, 3>(0, 15) = m_angOffset * skewSymmetric(
    m_posOffset);

  // IMU Positional Offset
  temp.setZero();
  temp(0, 0) = -(m_bodyAngVel(1) * m_bodyAngVel(1)) - (m_bodyAngVel(2) * m_bodyAngVel(2));
  temp(0, 1) = m_bodyAngVel(0) * m_bodyAngVel(1);
  temp(0, 2) = m_bodyAngVel(0) * m_bodyAngVel(2);
  temp(1, 0) = m_bodyAngVel(0) * m_bodyAngVel(1);
  temp(1, 1) = -(m_bodyAngVel(0) * m_bodyAngVel(0)) - (m_bodyAngVel(2) * m_bodyAngVel(2));
  temp(1, 2) = m_bodyAngVel(1) * m_bodyAngVel(2);
  temp(2, 0) = m_bodyAngVel(0) * m_bodyAngVel(2);
  temp(2, 1) = m_bodyAngVel(1) * m_bodyAngVel(2);
  temp(2, 2) = -(m_bodyAngVel(0) * m_bodyAngVel(0)) - (m_bodyAngVel(1) * m_bodyAngVel(1));
  measurementJacobian.block<3, 3>(0, 18) =
    m_angOffset * skewSymmetric(m_bodyAngAcc) + temp;

  // IMU Angular Offset
  Eigen::Vector3d imu_acc = m_bodyAcc +
    m_bodyAngAcc.cross(m_posOffset) +
    m_bodyAngVel.cross(m_bodyAngVel.cross(m_posOffset));

  measurementJacobian.block<3, 3>(0, 21) =
    -(m_angOffset * skewSymmetric(imu_acc));

  // IMU Body Angular Velocity
  measurementJacobian.block<3, 3>(3, 12) = m_angOffset.toRotationMatrix();

  // IMU Angular Offset
  measurementJacobian.block<3, 3>(3, 21) =
    -(m_angOffset * skewSymmetric(m_bodyAngVel));

  // IMU Accelerometer Bias
  measurementJacobian.block<3, 3>(0, 24) = Eigen::MatrixXd::Identity(3, 3);

  // IMU Gyroscope Bias
  measurementJacobian.block<3, 3>(3, 27) = Eigen::MatrixXd::Identity(3, 3);

  return measurementJacobian;
}

void ImuUpdater::RefreshStates()
{
  BodyState bodyState = m_ekf->getBodyState();
  m_bodyPos = bodyState.position;
  m_bodyVel = bodyState.velocity;
  m_bodyAcc = bodyState.acceleration;
  m_bodyAngPos = bodyState.orientation;
  m_bodyAngVel = bodyState.angularVelocity;
  m_bodyAngAcc = bodyState.angularAcceleration;

  ImuState imuState = m_ekf->getImuState(m_id);
  m_posOffset = imuState.position;
  m_angOffset = imuState.orientation;
  m_accBias = imuState.accBias;
  m_omgBias = imuState.omgBias;
}


/// @todo Move to a EKF updater class
/// @todo This is very slow for large states. Slice matrix before multiplications
void ImuUpdater::updateEKF(
  double time, Eigen::Vector3d acceleration,
  Eigen::Matrix3d accelerationCovariance, Eigen::Vector3d angularRate,
  Eigen::Matrix3d angularRateCovariance)
{
  RefreshStates();
  m_ekf->processModel(time);

  Eigen::VectorXd z(acceleration.size() + angularRate.size());
  z.segment<3>(0) = acceleration;
  z.segment<3>(3) = angularRate;

  Eigen::VectorXd z_pred = predictMeasurement();
  Eigen::VectorXd resid = z - z_pred;

  unsigned int stateSize = m_ekf->getState().getStateSize();
  unsigned int stateStartIndex = m_ekf->getImuStateStartIndex(m_id);

  Eigen::MatrixXd subH = getMeasurementJacobian();
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, stateSize);

  H.block<6, 18>(0, 0) = subH.block<6, 18>(0, 0);

  H.block(0, stateStartIndex, 6, 12) = subH.block<6, 12>(0, 18);

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(6, 6);
  R.block<3, 3>(0, 0) = minBoundDiagonal(accelerationCovariance, 1e-3);
  R.block<3, 3>(3, 3) = minBoundDiagonal(angularRateCovariance, 1e-2);

  Eigen::MatrixXd S = H * m_ekf->getCov() * H.transpose() + R;
  Eigen::MatrixXd K = m_ekf->getCov() * H.transpose() * S.inverse();

  Eigen::VectorXd update = K * resid;
  m_ekf->getState() += update;
  m_ekf->getCov() = (Eigen::MatrixXd::Identity(stateSize, stateSize) - K * H) * m_ekf->getCov();
}