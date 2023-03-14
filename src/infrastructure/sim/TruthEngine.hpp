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

#ifndef INFRASTRUCTURE__SIM__TRUTHENGINE_HPP_
#define INFRASTRUCTURE__SIM__TRUTHENGINE_HPP_

#include <eigen3/Eigen/Eigen>

///
/// @class TruthEngine
/// @brief Truth for low fidelity simulations
/// @todo Add initialization time to start of sim
///
class TruthEngine
{
public:
  TruthEngine() {}

  void SetBodyPosCycleFrequency(Eigen::Vector3d frequency);
  void SetBodyAngCycleFrequency(Eigen::Vector3d frequency);

  Eigen::Vector3d GetBodyPosition(double time);
  Eigen::Vector3d GetBodyVelocity(double time);
  Eigen::Vector3d GetBodyAcceleration(double time);

  Eigen::Quaterniond GetBodyAngularPosition(double time);
  Eigen::Vector3d GetBodyAngularRate(double time);
  Eigen::Vector3d GetBodyAngularAcceleration(double time);

private:
  Eigen::Vector3d m_posCycleFrequency {1.0, 1.0, 1.0};
  Eigen::Vector3d m_angCycleFrequency {1.0, 1.0, 1.0};
};

#endif  // INFRASTRUCTURE__SIM__TRUTHENGINE_HPP_
