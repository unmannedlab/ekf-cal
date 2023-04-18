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

#include <vector>

#include <std_msgs/msg/header.hpp>

#include "utility/TypeHelper.hpp"


TEST(test_TypeHelper, StdToEigVec) {
  std::vector<double> vec2 {1.0, 2.0};
  Eigen::VectorXd out2 = stdToEigVec(vec2);
  EXPECT_EQ(vec2.size(), out2.size());
  EXPECT_EQ(vec2[0], out2(0));
  EXPECT_EQ(vec2[1], out2(1));

  std::vector<double> vec3 {1.0, 2.0, 3.0};
  Eigen::Vector3d out3 = stdToEigVec(vec3);
  EXPECT_EQ(vec3.size(), out3.size());
  EXPECT_EQ(vec3[0], out3.x());
  EXPECT_EQ(vec3[1], out3.y());
  EXPECT_EQ(vec3[2], out3.z());
}

TEST(test_TypeHelper, StdToEigQuat) {
  std::vector<double> in {1.0, 2.0, 3.0, 4.0};
  double norm = sqrt(in[0] * in[0] + in[1] * in[1] + in[2] * in[2] + in[3] * in[3]);
  in[0] = in[0] / norm;
  in[1] = in[1] / norm;
  in[2] = in[2] / norm;
  in[3] = in[3] / norm;

  Eigen::Quaterniond out = stdToEigQuat(in);
  EXPECT_NEAR(out.w(), in[0], 1e-6);
  EXPECT_NEAR(out.x(), in[1], 1e-6);
  EXPECT_NEAR(out.y(), in[2], 1e-6);
  EXPECT_NEAR(out.z(), in[3], 1e-6);
}

TEST(test_TypeHelper, StdToEigQuat_err) {
  std::vector<double> in {1.0, 2.0, 3.0};

  Eigen::Quaterniond out = stdToEigQuat(in);
  EXPECT_EQ(out.w(), 1.0);
  EXPECT_EQ(out.x(), 0.0);
  EXPECT_EQ(out.y(), 0.0);
  EXPECT_EQ(out.z(), 0.0);
}

TEST(test_TypeHelper, RotVecToQuat) {
  Eigen::Vector3d vec0 {0.0, 0.0, 0.0};
  Eigen::Quaterniond quat0 = rotVecToQuat(vec0);
  EXPECT_NEAR(quat0.w(), 1.0000000, 1e-6);
  EXPECT_NEAR(quat0.x(), 0.0000000, 1e-6);
  EXPECT_NEAR(quat0.y(), 0.0000000, 1e-6);
  EXPECT_NEAR(quat0.z(), 0.0000000, 1e-6);

  Eigen::Vector3d vec1 {1.0, 0.0, 0.0};
  Eigen::Quaterniond quat1 = rotVecToQuat(vec1);
  EXPECT_NEAR(quat1.w(), 0.8775826, 1e-6);
  EXPECT_NEAR(quat1.x(), 0.4794255, 1e-6);
  EXPECT_NEAR(quat1.y(), 0.0000000, 1e-6);
  EXPECT_NEAR(quat1.z(), 0.0000000, 1e-6);

  Eigen::Vector3d vec2 {0.0, 1.0, 0.0};
  Eigen::Quaterniond quat2 = rotVecToQuat(vec2);
  EXPECT_NEAR(quat2.w(), 0.8775826, 1e-6);
  EXPECT_NEAR(quat2.x(), 0.0000000, 1e-6);
  EXPECT_NEAR(quat2.y(), 0.4794255, 1e-6);
  EXPECT_NEAR(quat2.z(), 0.0000000, 1e-6);

  Eigen::Vector3d vec3 {0.0, 0.0, 1.0};
  Eigen::Quaterniond quat3 = rotVecToQuat(vec3);
  EXPECT_NEAR(quat3.w(), 0.8775826, 1e-6);
  EXPECT_NEAR(quat3.x(), 0.0000000, 1e-6);
  EXPECT_NEAR(quat3.y(), 0.0000000, 1e-6);
  EXPECT_NEAR(quat3.z(), 0.4794255, 1e-6);

  Eigen::Vector3d vec4 {1.0, 2.0, 3.0};
  Eigen::Quaterniond quat4 = rotVecToQuat(vec4);
  EXPECT_NEAR(quat4.w(), -0.2955511, 1e-6);
  EXPECT_NEAR(quat4.x(), 0.2553219, 1e-6);
  EXPECT_NEAR(quat4.y(), 0.5106437, 1e-6);
  EXPECT_NEAR(quat4.z(), 0.7659656, 1e-6);

  Eigen::Vector3d vec5 {-4.0, 5.0, -6.0};
  Eigen::Quaterniond quat5 = rotVecToQuat(vec5);
  EXPECT_NEAR(quat5.w(), -0.3192205, 1e-6);
  EXPECT_NEAR(quat5.x(), 0.4319929, 1e-6);
  EXPECT_NEAR(quat5.y(), -0.5399911, 1e-6);
  EXPECT_NEAR(quat5.z(), 0.6479893, 1e-6);
}
