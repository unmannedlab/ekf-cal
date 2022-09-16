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

#include <gtest/gtest.h>
#include <iostream>
#include <vector>
#include <std_msgs/msg/header.hpp>

#include "RosHelper.hpp"

TEST(test_TypeHelper, RosHeaderToTime) {
  std_msgs::msg::Header in;
  in.stamp.nanosec = 987654321;
  in.stamp.sec = 123456789;
  double out = RosHelper::RosHeaderToTime(in);
  EXPECT_EQ(out, 123456789.987654321);
}

TEST(test_TypeHelper, RosVecToEigen) {
  geometry_msgs::msg::Vector3 in;
  in.x = 1.0;
  in.y = 2.0;
  in.z = 3.0;
  Eigen::Vector3d out = RosHelper::RosToEigen(in);
  EXPECT_EQ(out(0), in.x);
  EXPECT_EQ(out(1), in.y);
  EXPECT_EQ(out(2), in.z);
}

TEST(test_TypeHelper, RosMatToEigen) {
  std::array<double, 9> in{1, 2, 3, 4, 5, 6, 7, 8, 9};
  Eigen::Matrix3d out = RosHelper::RosToEigen(in);
  EXPECT_EQ(out(0, 0), in[0]);
  EXPECT_EQ(out(0, 1), in[1]);
  EXPECT_EQ(out(0, 2), in[2]);
  EXPECT_EQ(out(1, 0), in[3]);
  EXPECT_EQ(out(1, 1), in[4]);
  EXPECT_EQ(out(1, 2), in[5]);
  EXPECT_EQ(out(2, 0), in[6]);
  EXPECT_EQ(out(2, 1), in[7]);
  EXPECT_EQ(out(2, 2), in[8]);

  EXPECT_TRUE(true);
}
