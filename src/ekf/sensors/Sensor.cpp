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

#include "ekf/sensors/Sensor.hpp"

#include <string>

#include "TypeHelper.hpp"

// Initialize static variables
unsigned int Sensor::_idCount = 0;
Eigen::Vector3d Sensor::m_bodyPos(0.0, 0.0, 0.0);
Eigen::Vector3d Sensor::m_bodyVel(0.0, 0.0, 0.0);
Eigen::Vector3d Sensor::m_bodyAcc(0.0, 0.0, 0.0);
Eigen::Quaterniond Sensor::m_bodyAngPos(0.0, 0.0, 0.0, 0.0);
Eigen::Vector3d Sensor::m_bodyAngVel(0.0, 0.0, 0.0);
Eigen::Vector3d Sensor::m_bodyAngAcc(0.0, 0.0, 0.0);


Sensor::Sensor(std::string name)
: m_id(++_idCount), m_name(name) {}

std::string Sensor::GetName()
{
  return m_name;
}

unsigned int Sensor::GetId()
{
  return m_id;
}

unsigned int Sensor::GetStateStartIndex()
{
  return m_stateStartIndex;
}

unsigned int Sensor::GetStateSize()
{
  return m_stateSize;
}

void Sensor::SetStateStartIndex(unsigned int stateStartIndex)
{
  m_stateStartIndex = stateStartIndex;
}

Eigen::Vector3d Sensor::GetPosOffset()
{
  return m_posOffset;
}

void Sensor::SetPosOffset(Eigen::Vector3d posOffset)
{
  m_posOffset = posOffset;
}

Eigen::Quaterniond Sensor::GetAngOffset()
{
  return m_angOffset;
}

void Sensor::SetAngOffset(Eigen::Quaterniond angOffset)
{
  m_angOffset = angOffset;
}

/// @todo Implement check for correlation coefficients to be between +/- 1
void Sensor::SetCov(Eigen::MatrixXd cov)
{
  m_cov = cov;
}

Eigen::MatrixXd Sensor::GetCov()
{
  return m_cov;
}

void Sensor::SetBodyState(Eigen::VectorXd bodyState)
{
  m_bodyPos = bodyState.segment(0, 3);
  m_bodyVel = bodyState.segment(3, 3);
  m_bodyAcc = bodyState.segment(6, 3);
  m_bodyAngPos = TypeHelper::RotVecToQuat(bodyState.segment(9, 3));
  m_bodyAngVel = bodyState.segment(12, 3);
  m_bodyAngAcc = bodyState.segment(15, 3);
}
