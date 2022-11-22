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

#include "sensors/ros/RosCam.hpp"

#include "sensors/Sensor.hpp"
#include "utility/TypeHelper.hpp"

Camera::Camera(Camera::Params params)
: Sensor(params.name) {}

Eigen::VectorXd Camera::PredictMeasurement()
{
  Eigen::VectorXd predictedMeasurement(m_stateSize);
  return predictedMeasurement;
}

Eigen::MatrixXd Camera::GetMeasurementJacobian()
{
  Eigen::MatrixXd measurementJacobian(m_stateSize, m_stateSize);
  return measurementJacobian;
}

void Camera::SetState(Eigen::VectorXd state)
{
  m_posOffset = state.segment(0, 3);
  m_angOffset = TypeHelper::RotVecToQuat(state.segment(3, 3));
}

Eigen::VectorXd Camera::GetState()
{
  Eigen::AngleAxisd angAxis{m_angOffset};
  Eigen::Vector3d rotVec = angAxis.axis() * angAxis.angle();
  Eigen::VectorXd stateVec(m_posOffset.size() + rotVec.size());
  stateVec.segment<3>(0) = m_posOffset;
  stateVec.segment<3>(3) = rotVec;

  return stateVec;
}

void Camera::Callback(double time)
{
  m_Logger->log(LogLevel::INFO, "Camera callback not implemented");
}
