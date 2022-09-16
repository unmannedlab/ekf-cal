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

#include "ekf/EKF.hpp"
#include "CustomAssertions.hpp"

///
/// @todo Write test with varying covariance in sensors
///
TEST(test_EKF, register_imu) {
  EKF ekf;

  Imu::Params imu_params_base;
  imu_params_base.baseSensor = true;
  imu_params_base.intrinsic = false;

  Imu::Params imu_params_base_intrinsic;
  imu_params_base_intrinsic.baseSensor = true;
  imu_params_base_intrinsic.intrinsic = true;
  imu_params_base_intrinsic.variance = Eigen::VectorXd::Ones(6);

  Imu::Params imu_params_extrinsic;
  imu_params_extrinsic.baseSensor = false;
  imu_params_extrinsic.intrinsic = false;
  imu_params_extrinsic.variance = Eigen::VectorXd::Ones(6);

  Imu::Params imu_params_intrinsic;
  imu_params_intrinsic.baseSensor = false;
  imu_params_intrinsic.intrinsic = true;
  imu_params_intrinsic.variance = Eigen::VectorXd::Ones(12);

  EXPECT_EQ(ekf.GetStateSize(), 18U);
  EXPECT_EQ(ekf.GetState(), Eigen::VectorXd::Zero(18U));
  EXPECT_EQ(ekf.GetCov(), Eigen::MatrixXd::Identity(18U, 18U));

  unsigned int id_1 = ekf.RegisterSensor(imu_params_base);

  EXPECT_EQ(ekf.GetStateSize(), 18U);
  EXPECT_EQ(ekf.GetState(), Eigen::VectorXd::Zero(18U));
  EXPECT_EQ(ekf.GetCov(), Eigen::MatrixXd::Identity(18U, 18U));

  unsigned int id_2 = ekf.RegisterSensor(imu_params_base_intrinsic);

  EXPECT_EQ(ekf.GetStateSize(), 24U);
  EXPECT_EQ(ekf.GetState(), Eigen::VectorXd::Zero(24U));
  EXPECT_EQ(ekf.GetCov(), Eigen::MatrixXd::Identity(24U, 24U));

  unsigned int id_3 = ekf.RegisterSensor(imu_params_extrinsic);

  EXPECT_EQ(ekf.GetStateSize(), 30U);
  EXPECT_EQ(ekf.GetState(), Eigen::VectorXd::Zero(30U));
  EXPECT_EQ(ekf.GetCov(), Eigen::MatrixXd::Identity(30U, 30U));

  unsigned int id_4 = ekf.RegisterSensor(imu_params_intrinsic);

  EXPECT_EQ(ekf.GetStateSize(), 42U);
  EXPECT_EQ(ekf.GetState(), Eigen::VectorXd::Zero(42U));
  EXPECT_EQ(ekf.GetCov(), Eigen::MatrixXd::Identity(42U, 42U));

  EXPECT_EQ(id_1 + 1U, id_2);
  EXPECT_EQ(id_2 + 1U, id_3);
  EXPECT_EQ(id_3 + 1U, id_4);
}

TEST(test_EKF, GetStateTransition) {
  EKF ekf;

  // Identity Matrix (dt = 0)
  Eigen::MatrixXd phiOut1(18, 18);
  Eigen::MatrixXd phiExp1(18, 18);
  phiOut1 = ekf.GetStateTransition(0);
  phiExp1.setIdentity();

  EXPECT_TRUE(CustomAssertions::EXPECT_EIGEN_NEAR(phiOut1, phiExp1, 1e-6));


  // (dt = 0.25)
  Eigen::MatrixXd phiOut2(18, 18);
  Eigen::MatrixXd phiExp2(18, 18);
  phiOut2 = ekf.GetStateTransition(0.25);
  phiExp2.setIdentity();
  phiExp2.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * 0.25;
  phiExp2.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * 0.25;
  phiExp2.block<3, 3>(9, 12) = Eigen::Matrix3d::Identity() * 0.25;
  phiExp2.block<3, 3>(12, 15) = Eigen::Matrix3d::Identity() * 0.25;

  EXPECT_TRUE(CustomAssertions::EXPECT_EIGEN_NEAR(phiOut2, phiExp2, 1e-6));


  // Change state size
  Imu::Params params;
  params.baseSensor = false;
  params.intrinsic = true;
  params.variance = Eigen::VectorXd::Ones(12);
  ekf.RegisterSensor(params);

  Eigen::MatrixXd phiOut3(30, 30);
  Eigen::MatrixXd phiExp3(30, 30);
  phiOut3 = ekf.GetStateTransition(0.25);
  phiExp3.setIdentity();
  phiExp3.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * 0.25;
  phiExp3.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * 0.25;
  phiExp3.block<3, 3>(9, 12) = Eigen::Matrix3d::Identity() * 0.25;
  phiExp3.block<3, 3>(12, 15) = Eigen::Matrix3d::Identity() * 0.25;

  EXPECT_TRUE(CustomAssertions::EXPECT_EIGEN_NEAR(phiOut3, phiExp3, 1e-6));
}

TEST(test_EKF, GetProcessInput) {
  EKF ekf;

  EXPECT_EQ(ekf.GetProcessInput(), Eigen::MatrixXd::Identity(18, 18));

  Imu::Params params;
  params.baseSensor = false;
  params.intrinsic = true;
  params.variance = Eigen::VectorXd::Ones(12);
  ekf.RegisterSensor(params);

  EXPECT_EQ(ekf.GetProcessInput(), Eigen::MatrixXd::Identity(30, 30));
}

TEST(test_EKF, GetProcessNoise) {
  EKF ekf;

  EXPECT_EQ(ekf.GetProcessNoise(), Eigen::MatrixXd::Zero(18, 18));

  Imu::Params params;
  params.baseSensor = false;
  params.intrinsic = true;
  params.variance = Eigen::VectorXd::Ones(12);
  params.accBiasStability = 0.001;
  params.omgBiasStability = 0.01;
  ekf.RegisterSensor(params);

  Eigen::MatrixXd noiseOut(30, 30);
  Eigen::MatrixXd noiseExp(30, 30);

  noiseOut = ekf.GetProcessNoise();
  noiseExp.setZero();
  noiseExp.block<3, 3>(24, 24) = Eigen::Matrix3d::Identity() * params.accBiasStability;
  noiseExp.block<3, 3>(27, 27) = Eigen::Matrix3d::Identity() * params.omgBiasStability;

  EXPECT_EQ(noiseOut, noiseExp);
}

TEST(test_EKF, Predict) {
  // Initialize
  EKF ekf;
  Eigen::VectorXd bodyState(18);
  bodyState << 1, 1, 1, 2, 2, 2, 3, 3, 3, 0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 0.3, 0.3, 0.3;
  ekf.InitializeBodyState(0.0, bodyState);

  Eigen::VectorXd stateExp(18);
  Eigen::VectorXd stateOut(18);
  Eigen::MatrixXd covExp(18, 18);
  Eigen::MatrixXd covOut(18, 18);

  // Test past
  ekf.Predict(-0.25);

  stateOut = ekf.GetState();
  covOut = ekf.GetCov();

  stateExp = bodyState;
  covExp.setIdentity();

  EXPECT_TRUE(CustomAssertions::EXPECT_EIGEN_NEAR(stateExp, stateOut, 1e-6));
  EXPECT_TRUE(CustomAssertions::EXPECT_EIGEN_NEAR(covExp, covOut, 1e-6));

  // Test future
  ekf.Predict(0.25);

  stateOut = ekf.GetState();
  covOut = ekf.GetCov();

  stateExp.setZero();
  covExp.setZero();

  stateExp.segment<3>(0) << 1.500, 1.500, 1.500;
  stateExp.segment<3>(3) << 2.750, 2.750, 2.750;
  stateExp.segment<3>(6) << 3.000, 3.000, 3.000;
  stateExp.segment<3>(9) << 0.150, 0.150, 0.150;
  stateExp.segment<3>(12) << 0.275, 0.275, 0.275;
  stateExp.segment<3>(15) << 0.300, 0.300, 0.300;

  covExp.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1.0625;
  covExp.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1.0625;
  covExp.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
  covExp.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 1.0625;
  covExp.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 1.0625;
  covExp.block<3, 3>(15, 15) = Eigen::Matrix3d::Identity();

  covExp.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * 0.25;
  covExp.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * 0.25;
  covExp.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * 0.25;
  covExp.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * 0.25;

  covExp.block<3, 3>(9, 12) = Eigen::Matrix3d::Identity() * 0.25;
  covExp.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity() * 0.25;
  covExp.block<3, 3>(12, 15) = Eigen::Matrix3d::Identity() * 0.25;
  covExp.block<3, 3>(15, 12) = Eigen::Matrix3d::Identity() * 0.25;

  std::cout << std::setprecision(9) << stateOut << std::endl;
  std::cout << std::setprecision(9) << covOut << std::endl;

  EXPECT_TRUE(CustomAssertions::EXPECT_EIGEN_NEAR(stateExp, stateOut, 1e-6));
  EXPECT_TRUE(CustomAssertions::EXPECT_EIGEN_NEAR(covExp, covOut, 1e-6));
}
