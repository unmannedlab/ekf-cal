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

#ifndef MATHHELPER_HPP_
#define MATHHELPER_HPP_

#include <eigen3/Eigen/Eigen>

namespace MathHelper
{
///
/// @brief Produces a cross product matrix
/// @param inVec Input vector with which to find the left hand size cross
/// product matrix
/// @return Cross product matrix
///
inline Eigen::Matrix3d CrossProductMatrix(Eigen::Vector3d inVec)
{
  Eigen::Matrix3d outMat = Eigen::Matrix3d::Zero();

  outMat(0U, 1U) = -inVec(2U);
  outMat(0U, 2U) = inVec(1U);
  outMat(1U, 2U) = -inVec(0U);
  outMat(1U, 0U) = inVec(2U);
  outMat(2U, 0U) = -inVec(1U);
  outMat(2U, 1U) = inVec(0U);

  return outMat;
}

}  // namespace MathHelper

#endif  // MATHHELPER_HPP_
