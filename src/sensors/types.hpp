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

#ifndef SENSORS__TYPES_HPP_
#define SENSORS__TYPES_HPP_

enum class SensorType
{
  IMU,
  Camera,
  Tracker
};

typedef struct Intrinsics
{
  double f_x {1.0};           ///< @brief X focal length
  double f_y {1.0};           ///< @brief Y focal length
  double c_x {0.0};           ///< @brief X optical center
  double c_y {0.0};           ///< @brief Y optical center
  double k_1 {0.0};           ///< @brief Radial coefficient 1
  double k_2 {0.0};           ///< @brief Radial coefficient 2
  double p_1 {0.0};           ///< @brief Tangential coefficient 1
  double p_2 {0.0};           ///< @brief Tangential coefficient 1
  double pixel_size {0.010};  ///< @brief Pixel size
} Intrinsics;

#endif  // SENSORS__TYPES_HPP_
