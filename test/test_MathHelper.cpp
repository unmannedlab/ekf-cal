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

#include <eigen3/Eigen/Eigen>

#include <gtest/gtest.h>
#include <iostream>

#include "MathHelper.hpp"

TEST(test_MathHelper, CrossProductMatrix) {
  Eigen::Vector3d testVec(1.0, 2.0, 3.0);
  Eigen::Matrix3d outMat = MathHelper::CrossProductMatrix(testVec);
  EXPECT_EQ(outMat(0, 0), 0.0);
  EXPECT_EQ(outMat(0, 1), -testVec(2));
  EXPECT_EQ(outMat(0, 2), testVec(1));
  EXPECT_EQ(outMat(1, 2), -testVec(0));
  EXPECT_EQ(outMat(1, 1), 0.0);
  EXPECT_EQ(outMat(1, 0), testVec(2));
  EXPECT_EQ(outMat(2, 0), -testVec(1));
  EXPECT_EQ(outMat(2, 1), testVec(0));
  EXPECT_EQ(outMat(2, 2), 0.0);
}
