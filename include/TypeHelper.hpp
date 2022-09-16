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

#ifndef TYPEHELPER_HPP_
#define TYPEHELPER_HPP_

#include <eigen3/Eigen/Eigen>

#include <vector>

#include "Constants.hpp"

namespace TypeHelper
{

///
/// @brief Converts std::vector into Eigen Quaternion
/// @param in Input std::vector
/// @return Output Eigen Vector3
///
inline Eigen::VectorXd StdToEigVec(std::vector<double> const & in)
{
  Eigen::VectorXd out(in.size());
  for (unsigned int i = 0; i < in.size(); ++i) {
    out(i) = in[i];
  }
  return out;
}

///
/// @brief Converts std::vector into Eigen Quaternion
/// @param in Input std::vector
/// @return Output Eigen Quaternion
///
inline Eigen::Quaterniond StdToEigQuat(std::vector<double> const & in)
{
  if (in.size() == 4U) {
    Eigen::Quaterniond quat{in[0U], in[1U], in[2U], in[3U]};
    quat.normalize();
    return quat;
  } else {
    /// @todo replace with generic logging
    // RCLCPP_WARN(
    //   rclcpp::get_logger(
    //     "TypeHelper"), "Vector incorrect size for Eigen conversion. Size: '%u'", in.size());

    return Eigen::Quaterniond{1.0, 0.0, 0.0, 0.0};
  }
}

inline Eigen::Quaterniond RotVecToQuat(Eigen::Vector3d rotVec)
{
  double angle = rotVec.norm();
  if (angle > 1e-9) {
    Eigen::Vector3d axis = rotVec / rotVec.norm();
    Eigen::AngleAxisd angAxis{angle, axis};
    return Eigen::Quaterniond(angAxis);
  } else {
    return Eigen::Quaterniond{1, 0, 0, 0};
  }
}

}  // namespace TypeHelper

#endif  // TYPEHELPER_HPP_
