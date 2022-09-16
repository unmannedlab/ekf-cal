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

#ifndef ROSHELPER_HPP_
#define ROSHELPER_HPP_

#include <eigen3/Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <std_msgs/msg/header.hpp>
#include <vector>

#include "Constants.hpp"

namespace RosHelper
{

inline double RosHeaderToTime(std_msgs::msg::Header header)
{
  return header.stamp.sec + (header.stamp.nanosec * NSEC_TO_SEC);
}

inline Eigen::Vector3d RosToEigen(geometry_msgs::msg::Vector3 msg)
{
  return Eigen::Vector3d {msg.x, msg.y, msg.z};
}

inline Eigen::Matrix3d RosToEigen(std::array<double, 9> msg)
{
  return Eigen::Matrix3d {
    {msg[0], msg[1], msg[2]},
    {msg[3], msg[4], msg[5]},
    {msg[6], msg[7], msg[8]}
  };
}

}  // namespace RosHelper

#endif  // ROSHELPER_HPP_
